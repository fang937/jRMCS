#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <string>

#include <librmcs/board/c_board.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <librmcs/data/datas.hpp>
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"

#include "hardware/device/c_board_button.hpp"
#include "hardware/device/c_board_servo.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/remote_control.hpp"

namespace rmcs_core::hardware {

class DartLauncherHardware
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public librmcs::board::CBoard::Callback {
public:
    DartLauncherHardware()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , command_component_(
              create_partner_component<Command>(get_component_name() + "_command", *this)) {
        register_output("/dart/button/fire", fire_button_output_, false);
        register_output("/dart/button/reload", reload_button_output_, false);
        register_output("/dart/button/reload_cancel", reload_cancel_button_output_, false);
        for (std::size_t i = 0; i < servos_.size(); ++i)
            command_component_->register_input(
                "/dart/servo/" + std::to_string(i + 1) + "/angle", servo_angles_[i], false);

        RCLCPP_INFO(
            get_logger(),
            "Motor mapping (reATRM): CAN1 command=0x%03X; "
            "left M3508 ID%u feedback=0x%03X slot1; right M3508 ID%u feedback=0x%03X slot2; "
            "yaw M2006 ID%u feedback=0x%03X slot3; distance M2006 ID%u feedback=0x%03X slot4",
            static_cast<unsigned>(yaw_motor_.send_id()),
            static_cast<unsigned>(trigger_motors_[0].id()), static_cast<unsigned>(trigger_motors_[0].recv_id()),
            static_cast<unsigned>(trigger_motors_[1].id()), static_cast<unsigned>(trigger_motors_[1].recv_id()),
            static_cast<unsigned>(yaw_motor_.id()), static_cast<unsigned>(yaw_motor_.recv_id()),
            static_cast<unsigned>(distance_motor_.id()), static_cast<unsigned>(distance_motor_.recv_id()));

        board_ = std::make_unique<librmcs::board::CBoard>(*this, get_parameter("board_serial").as_string());
        remote_control_ = std::make_unique<device::RemoteControl>(*this);
        remote_control_->register_dr16(&dr16_);
        auto builder = board_->start_transmit();
        servos_[0].enable(Spec::kGpios.kPwm1);
        servos_[1].enable(Spec::kGpios.kPwm2);
        servos_[2].enable(Spec::kGpios.kPwm3);
        servos_[3].enable(Spec::kGpios.kPwm4);
        buttons_[0].enable(builder, Spec::kGpios.kPwm5);
        buttons_[1].enable(builder, Spec::kGpios.kPwm6);
        buttons_[2].enable(builder, Spec::kGpios.kPwm7);
        for (auto& servo : servos_)
            servo.write(builder);
        RCLCPP_INFO(
            get_logger(),
            "Dart launcher board initialized: serial='%s', initial angles=[%.1f, %.1f, %.1f, %.1f]",
            get_parameter("board_serial").as_string().c_str(), servos_[0].angle(), servos_[1].angle(),
            servos_[2].angle(), servos_[3].angle());
    }

    void update() override {
        dr16_.update_status();
        remote_control_->update();
        for (auto& motor : trigger_motors_)
            motor.update_status();
        yaw_motor_.update_status();
        distance_motor_.update_status();
        *fire_button_output_ = buttons_[0].pressed();
        *reload_button_output_ = buttons_[1].pressed();
        *reload_cancel_button_output_ = buttons_[2].pressed();
    }

    void gpio_digital_read_result_callback(const Spec::Gpio& gpio, const View::GpioDigital& data) override {
        for (auto& button : buttons_)
            if (button.handles(gpio))
                button.handle_read_result(data);
    }

    void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission || data.can_data.size() < 8)
            return;
        const std::size_t bus = can == Spec::kCans.kCan1 ? 0 : 1;
        can_feedback_count_[bus].fetch_add(1, std::memory_order_relaxed);
        last_can_id_[bus].store(data.can_id, std::memory_order_relaxed);
        if (data.can_id >= 0x201 && data.can_id <= 0x208)
            dji_feedback_mask_[bus].fetch_or(1U << (data.can_id - 0x201), std::memory_order_relaxed);
        if (can == Spec::kCans.kCan1) {
            for (auto& motor : trigger_motors_)
                motor.match_then_store_status(data.can_id, data.can_data);
            if (yaw_motor_.match_then_store_status(data.can_id, data.can_data))
                yaw_feedback_count_.fetch_add(1, std::memory_order_relaxed);
            if (distance_motor_.match_then_store_status(data.can_id, data.can_data))
                distance_feedback_count_.fetch_add(1, std::memory_order_relaxed);
        }
    }

    void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
        if (uart == Spec::kUarts.kDbus)
            dr16_.store_status(data.uart_data.data(), data.uart_data.size());
    }

    void command_update() {
        auto builder = board_->start_transmit();
        bool all_ready = true;
        for (std::size_t i = 0; i < servos_.size(); ++i) {
            if (!servo_angles_[i].ready()) {
                all_ready = false;
                continue;
            }
            servos_[i].set_angle(builder, *servo_angles_[i]);
        }
        if (!all_ready)
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Servo command inputs are not ready; skipped unavailable channels");
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Servo command transmit: ready=%s angles=[%.1f, %.1f, %.1f, %.1f] duty16=[%u, %u, %u, %u]",
            all_ready ? "yes" : "no", servos_[0].angle(), servos_[1].angle(), servos_[2].angle(),
            servos_[3].angle(), static_cast<unsigned>(servos_[0].duty16()),
            static_cast<unsigned>(servos_[1].duty16()), static_cast<unsigned>(servos_[2].duty16()),
            static_cast<unsigned>(servos_[3].duty16()));
        const auto yaw_command = yaw_motor_.generate_command();
        const auto distance_command = distance_motor_.generate_command();
        const auto yaw_feedback = yaw_feedback_count_.exchange(0, std::memory_order_relaxed);
        const auto distance_feedback = distance_feedback_count_.exchange(0, std::memory_order_relaxed);
        yaw_feedback_since_log_ += yaw_feedback;
        distance_feedback_since_log_ += distance_feedback;
        const auto now = std::chrono::steady_clock::now();
        if (now - last_motor_log_ >= std::chrono::seconds(1)) {
            for (std::size_t bus = 0; bus < 2; ++bus)
                RCLCPP_INFO(
                    get_logger(),
                    "CAN%zu RX: frames=%u last_id=0x%03X DJI_ID_mask=0x%02X (bit0..7 = motor ID1..8)",
                    bus + 1, can_feedback_count_[bus].exchange(0, std::memory_order_relaxed),
                    last_can_id_[bus].load(std::memory_order_relaxed),
                    dji_feedback_mask_[bus].exchange(0, std::memory_order_relaxed));
            RCLCPP_INFO(
                get_logger(),
                "CAN1 0x200 motors ID3/4: torque=[%.3f, %.3f] angle=[%.3f, %.3f] feedback_frames=[%u, %u]",
                yaw_motor_.control_torque(), distance_motor_.control_torque(),
                yaw_motor_.angle(), distance_motor_.angle(),
                yaw_feedback_since_log_, distance_feedback_since_log_);
            if (yaw_feedback_since_log_ == 0 || distance_feedback_since_log_ == 0)
                RCLCPP_WARN(
                    get_logger(), "Motor feedback missing: expected CAN1 yaw=0x203 distance=0x204; check motor IDs, power and CAN wiring");
            yaw_feedback_since_log_ = distance_feedback_since_log_ = 0;
            last_motor_log_ = now;
        }
        // M3508/M2006 IDs 1..4 occupy the four slots of the same 0x200 frame.
        builder
            .can_transmit(Spec::kCans.kCan1, {
                .can_id = 0x200,
                .can_data = device::CanPacket8{
                    trigger_motors_[0].generate_command(), trigger_motors_[1].generate_command(),
                    yaw_command, distance_command}
                                .as_bytes(),
            });
    }

private:
    class Command final : public rmcs_executor::Component {
    public:
        explicit Command(DartLauncherHardware& hardware)
            : hardware_(hardware) {}

        void update() override { hardware_.command_update(); }

    private:
        DartLauncherHardware& hardware_;
    };

    using Spec = librmcs::board::CBoard::Callback::Spec;
    using View = librmcs::board::CBoard::Callback::View;

    std::unique_ptr<librmcs::board::CBoard> board_;
    std::shared_ptr<Command> command_component_;
    std::unique_ptr<device::RemoteControl> remote_control_;
    device::Dr16 dr16_;
    std::array<std::atomic<unsigned>, 2> can_feedback_count_{};
    std::array<std::atomic<unsigned>, 2> last_can_id_{};
    std::array<std::atomic<unsigned>, 2> dji_feedback_mask_{};
    std::atomic<unsigned> yaw_feedback_count_{0}, distance_feedback_count_{0};
    unsigned yaw_feedback_since_log_ = 0, distance_feedback_since_log_ = 0;
    std::chrono::steady_clock::time_point last_motor_log_ = std::chrono::steady_clock::now();
    std::array<device::CBoardServo, 4> servos_;
    std::array<device::CBoardButton, 3> buttons_;
    device::DjiMotor trigger_motors_[2]{
        {*this, *command_component_, "/trigger/left_motor", device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 1}.set_encoder_zero_point(static_cast<int>(get_parameter("left_motor_zero_point").as_int())).enable_multi_turn_angle()},
        {*this, *command_component_, "/trigger/right_motor", device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 2}.set_encoder_zero_point(static_cast<int>(get_parameter("right_motor_zero_point").as_int())).enable_multi_turn_angle()},
    };
    device::DjiMotor yaw_motor_{*this, *command_component_, "/yaw_motor", device::DjiMotor::Config{device::DjiMotor::Type::kM2006, 3}.set_encoder_zero_point(static_cast<int>(get_parameter("yaw_motor_zero_point").as_int())).enable_multi_turn_angle()};
    device::DjiMotor distance_motor_{*this, *command_component_, "/distance_motor", device::DjiMotor::Config{device::DjiMotor::Type::kM2006, 4}.set_encoder_zero_point(static_cast<int>(get_parameter("distance_motor_zero_point").as_int())).enable_multi_turn_angle()};
    InputInterface<double> servo_angles_[4];
    OutputInterface<bool> fire_button_output_;
    OutputInterface<bool> reload_button_output_;
    OutputInterface<bool> reload_cancel_button_output_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::DartLauncherHardware, rmcs_executor::Component)
