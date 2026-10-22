#include <array>
#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>

#include <eigen3/Eigen/Dense>
#include <librmcs/board/c_board.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/remote_control.hpp"

namespace rmcs_core::hardware {

class DartLauncher
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public librmcs::board::CBoard::Callback {
    using Board = librmcs::board::CBoard;
    using Can = librmcs::board::CBoard::Callback::Spec::Can;
    using Uart = librmcs::board::CBoard::Callback::Spec::Uart;

public:
    DartLauncher()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , command_(create_partner_component<Command>(get_component_name() + "_command", *this))
        , trigger_motor_{
              {*this, *command_, "/trigger/left_motor",
               device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 1}
                   .set_encoder_zero_point(get_parameter("left_motor_zero_point").as_int())
                   .enable_multi_turn_angle()},
              {*this, *command_, "/trigger/right_motor",
               device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 2}
                   .set_encoder_zero_point(get_parameter("right_motor_zero_point").as_int())
                   .enable_multi_turn_angle()}}
        , launcher_motor_{
              {*this, *command_, "/yaw_motor",
               device::DjiMotor::Config{device::DjiMotor::Type::kM2006, 3}
                   .set_encoder_zero_point(get_parameter("yaw_motor_zero_point").as_int())
                   .enable_multi_turn_angle()},
              {*this, *command_, "/distance_motor",
               device::DjiMotor::Config{device::DjiMotor::Type::kM2006, 4}
                   .set_encoder_zero_point(get_parameter("distance_motor_zero_point").as_int())
                   .enable_multi_turn_angle()}}
        , remote_control_(*this) {
        remote_control_.register_dr16(&dr16_);
        for (std::size_t index = 0; index < std::size(servo_angle_); ++index)
            register_input(
                "/trigger/servo/servo_" + std::to_string(index + 1) + "/angle",
                servo_angle_[index], 0.0);
        register_input("/trigger/left_motor/release", left_release_, false);
        register_input("/trigger/right_motor/release", right_release_, false);
        register_input("/trigger/servo/fire_request_sequence", fire_request_sequence_, 0);
        register_output("/trigger/servo/response_sequence", response_sequence_, 0);
        register_output("/trigger/servo/response_command", response_command_, 0);
        register_output("/trigger/servo/response_status", response_status_, 0);
        register_output("/trigger/servo/response_id", response_id_, 0);
        board_ = std::make_unique<Board>(*this, get_parameter("board_serial").as_string());
    }

    void update() override {
        for (auto& motor : trigger_motor_)
            motor.update_status();
        for (auto& motor : launcher_motor_)
            motor.update_status();
        dr16_.update_status();
        remote_control_.update();
    }

private:
    class Command final : public rmcs_executor::Component {
    public:
        explicit Command(DartLauncher& owner)
            : owner_(owner) {}

        void update() override { owner_.command_update(); }

    private:
        DartLauncher& owner_;
    };

    void command_update() {
        auto builder = board_->start_transmit();
        const auto fire_request = fire_request_sequence_.ready() ? *fire_request_sequence_ : 0;
        const bool fire_request_pending = fire_request != last_fire_request_sequence_;
        const auto left_command = left_release_.ready() && *left_release_
            ? trigger_motor_[0].generate_command(0.0)
            : trigger_motor_[0].generate_command();
        const auto right_command = right_release_.ready() && *right_release_
            ? trigger_motor_[1].generate_command(0.0)
            : trigger_motor_[1].generate_command();
        auto packet = device::CanPacket8{
            left_command, right_command, launcher_motor_[0].generate_command(),
            launcher_motor_[1].generate_command()};
        builder.can_transmit(
            librmcs::spec::c_board::kCanDescriptors.kCan1,
            {.can_id = 0x200, .can_data = packet.as_bytes()});

        for (std::size_t index = 0; index < std::size(servo_angle_); ++index) {
            const auto angle = std::clamp(
                std::isfinite(*servo_angle_[index]) ? *servo_angle_[index] : 0.0, 0.0, 180.0);
            const auto pulse_us = static_cast<std::uint32_t>(500 + angle * 2000.0 / 180.0);
            const auto duty = static_cast<std::uint16_t>(
                (static_cast<std::uint64_t>(pulse_us) * 65535 + 10000) / 20000);
            builder.gpio_analog_write(
                librmcs::spec::c_board::kGpioDescriptors[index], {.value = duty});
            if (angle != last_servo_angle_[index] || (index == 0 && fire_request_pending)) {
                last_servo_angle_[index] = angle;
                *response_sequence_ = static_cast<std::uint8_t>(*response_sequence_ + 1);
                *response_command_ = 0x81;
                *response_status_ = 0;
                *response_id_ = static_cast<std::uint8_t>(index + 1);
            }
        }
        if (fire_request_pending)
            last_fire_request_sequence_ = fire_request;
    }

    void can_receive_callback(const Can& can, const View::Can& data) override {
        if (can != librmcs::spec::c_board::kCanDescriptors.kCan1
            || data.is_extended_can_id || data.is_remote_transmission
            || data.can_data.size() != 8)
            return;
        for (auto& motor : trigger_motor_)
            if (motor.match_then_store_status(data.can_id, data.can_data))
                return;
        for (auto& motor : launcher_motor_)
            if (motor.match_then_store_status(data.can_id, data.can_data))
                return;
    }

    void uart_receive_callback(const Uart& uart, const View::Uart& data) override {
        if (uart == librmcs::spec::c_board::kUartDescriptors.kDbus)
            dr16_.store_status(data.uart_data.data(), data.uart_data.size());
    }

    std::shared_ptr<Command> command_;
    std::unique_ptr<Board> board_;
    device::DjiMotor trigger_motor_[2];
    device::DjiMotor launcher_motor_[2];
    device::Dr16 dr16_;
    device::RemoteControl remote_control_;
    InputInterface<double> servo_angle_[4];
    InputInterface<bool> left_release_, right_release_;
    InputInterface<std::uint8_t> fire_request_sequence_;
    OutputInterface<std::uint8_t> response_sequence_;
    OutputInterface<std::uint8_t> response_command_;
    OutputInterface<std::uint8_t> response_status_;
    OutputInterface<std::uint8_t> response_id_;
    std::array<double, 4> last_servo_angle_{-1.0, -1.0, -1.0, -1.0};
    std::uint8_t last_fire_request_sequence_ = 0;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::DartLauncher, rmcs_executor::Component)
