#include <array>
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
        if (can == Spec::kCans.kCan1) {
            for (auto& motor : trigger_motors_)
                motor.match_then_store_status(data.can_id, data.can_data);
            yaw_motor_.match_then_store_status(data.can_id, data.can_data);
            distance_motor_.match_then_store_status(data.can_id, data.can_data);
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
        builder
            .can_transmit(Spec::kCans.kCan1, {
                .can_id = 0x200,
                .can_data = device::CanPacket8{
                    trigger_motors_[0].generate_command(), trigger_motors_[1].generate_command(),
                    device::CanPacket8::PaddingQuarter{}, device::CanPacket8::PaddingQuarter{}}
                                .as_bytes(),
            })
            .can_transmit(Spec::kCans.kCan1, {
                .can_id = 0x1FF,
                .can_data = device::CanPacket8{
                    device::CanPacket8::PaddingQuarter{}, device::CanPacket8::PaddingQuarter{},
                    yaw_motor_.generate_command(), distance_motor_.generate_command()}
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
    std::array<device::CBoardServo, 4> servos_;
    std::array<device::CBoardButton, 3> buttons_;
    device::DjiMotor trigger_motors_[2]{
        {*this, *command_component_, "/trigger/left_motor", device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 1}.enable_multi_turn_angle()},
        {*this, *command_component_, "/trigger/right_motor", device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 2}.enable_multi_turn_angle()},
    };
    device::DjiMotor yaw_motor_{*this, *command_component_, "/yaw_motor", device::DjiMotor::Config{device::DjiMotor::Type::kM2006, 3}.enable_multi_turn_angle()};
    device::DjiMotor distance_motor_{*this, *command_component_, "/distance_motor", device::DjiMotor::Config{device::DjiMotor::Type::kM2006, 4}.enable_multi_turn_angle()};
    InputInterface<double> servo_angles_[4];
    OutputInterface<bool> fire_button_output_;
    OutputInterface<bool> reload_button_output_;
    OutputInterface<bool> reload_cancel_button_output_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::DartLauncherHardware, rmcs_executor::Component)
