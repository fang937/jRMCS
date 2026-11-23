#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <stdexcept>
#include <vector>

#include <librmcs/board/c_board.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <rmcs_executor/component.hpp>
#include <librmcs/data/datas.hpp>
#include "hardware/device/bmi088_ekf.hpp"
#include "hardware/device/board_clock_lifter.hpp"
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
              create_partner_component<Command>(get_component_name() + "_command", *this))
        , dynamic_tf_{this}
        , static_tf_{this} {
        register_output("/dart/motors/feedback_ready", motor_feedback_ready_, false);
        register_output("/yaw_motor/feedback_ready", yaw_motor_feedback_ready_, false);
        register_output("/distance_motor/feedback_ready", distance_motor_feedback_ready_, false);
        register_output("/dart/trigger_pair/feedback_ready", trigger_pair_feedback_ready_, false);
        register_output("/dart/button/fire", fire_button_output_, false);
        register_output("/dart/button/reload", reload_button_output_, false);
        register_output("/dart/button/reload_cancel", reload_cancel_button_output_, false);
        for (std::size_t i = 0; i < servos_.size(); ++i)
            command_component_->register_input(
                "/dart/servo/" + std::to_string(i + 1) + "/angle", servo_angles_[i], false);

        initialize_imu_tf();

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
        RCLCPP_INFO(get_logger(), "Dart launcher board initialized with four PWM servos");
    }

    ~DartLauncherHardware() override { board_.reset(); }

    void update() override {
        dr16_.update_status();
        remote_control_->update();
        // Each motor is enabled independently once its own feedback is observed.
        const auto feedback_mask = motor_feedback_mask_.load(std::memory_order_acquire);
        const bool feedback_ready = (feedback_mask & 0x0FU) == 0x0FU;
        for (auto& motor : trigger_motors_)
            motor.update_status();
        yaw_motor_.update_status();
        distance_motor_.update_status();
        *motor_feedback_ready_ = feedback_ready;
        *yaw_motor_feedback_ready_ = (feedback_mask & (1U << 2)) != 0;
        *distance_motor_feedback_ready_ = (feedback_mask & (1U << 3)) != 0;
        *trigger_pair_feedback_ready_ = (feedback_mask & 0x03U) == 0x03U;
        *fire_button_output_ = buttons_[0].pressed();
        *reload_button_output_ = buttons_[1].pressed();
        *reload_cancel_button_output_ = buttons_[2].pressed();
        publish_imu_tf();
    }

    void gpio_digital_read_result_callback(const Spec::Gpio& gpio, const View::GpioDigital& data) override {
        for (auto& button : buttons_)
            if (button.handles(gpio))
                button.handle_read_result(data);
    }

    void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission || data.can_data.size() != 8)
            return;
        if (can == Spec::kCans.kCan1) {
            unsigned received = 0;
            for (std::size_t i = 0; i < 2; ++i)
                if (trigger_motors_[i].match_then_store_status(data.can_id, data.can_data))
                    received |= 1U << i;
            if (yaw_motor_.match_then_store_status(data.can_id, data.can_data))
                received |= 1U << 2;
            if (distance_motor_.match_then_store_status(data.can_id, data.can_data))
                received |= 1U << 3;
            motor_feedback_mask_.fetch_or(received, std::memory_order_release);
        }
    }

    void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
        if (uart == Spec::kUarts.kDbus)
            dr16_.store_status(data.uart_data.data(), data.uart_data.size());
    }

    void accelerometer_receive_callback(const View::ImuAccelerometer& data) override {
        const auto timestamp = imu_clock_.advance_timebase(data.timestamp_quarter_us);
        imu_.push_accelerometer_sample(data.x, data.y, data.z, timestamp);
        // Match Bmi088Ekf's +/-6 g scaling; ROS acceleration uses m/s^2.
        latest_acceleration_ = Eigen::Vector3d{static_cast<double>(data.x),
            static_cast<double>(data.y), static_cast<double>(data.z)} * (6.0 * 9.80665 / 32767.0);
    }

    void gyroscope_receive_callback(const View::ImuGyroscope& data) override {
        const auto timestamp = imu_clock_.lift_timestamp(data.timestamp_quarter_us);
        if (!timestamp)
            return;
        if (const auto snapshot = imu_.try_update_with_gyroscope_sample(data.x, data.y, data.z, *timestamp)) {
            const std::scoped_lock lock{imu_mutex_};
            imu_state_ = ImuState{*snapshot, latest_acceleration_};
        }
    }

    void command_update() {
        auto builder = board_->start_transmit();
        const auto feedback_mask = motor_feedback_mask_.load(std::memory_order_acquire);
        // PWM servos only need a 50 Hz refresh. Keep CAN motor commands at the
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Trigger CAN1 feedback seen: left(0x201)=%s right(0x202)=%s; "
            "requested torque left=%.3f right=%.3f Nm",
            (feedback_mask & 1U) ? "yes" : "no",
            (feedback_mask & 2U) ? "yes" : "no",
            trigger_motors_[0].control_torque(), trigger_motors_[1].control_torque());
        // executor rate without generating a USB GPIO write on every 1 ms tick.
        const bool update_servos = servo_update_ticks_++ % kServoUpdateDivider == 0;
        if (update_servos) {
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
        }
        const auto command = [](const device::DjiMotor& motor, bool ready) {
            return motor.generate_command(ready ? motor.control_torque() : 0.0);
        };
        // Each motor is gated by its own feedback bit. A missing motor receives
        // zero current while other motors remain independently controllable.
        builder
            .can_transmit(Spec::kCans.kCan1, {
                .can_id = 0x200,
                .can_data = device::CanPacket8{
                    command(trigger_motors_[0], feedback_mask & (1U << 0)),
                    command(trigger_motors_[1], feedback_mask & (1U << 1)),
                    command(yaw_motor_, feedback_mask & (1U << 2)),
                    command(distance_motor_, feedback_mask & (1U << 3))}
                                .as_bytes(),
            });
    }

private:
    void initialize_imu_tf() {
        imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>("/dart/imu/data", rclcpp::SensorDataQoS{});
        const auto rate = get_parameter("publish_rate").as_double();
        if (!std::isfinite(rate) || rate <= 0.0 || rate > 1000.0)
            throw std::runtime_error{"publish_rate must be in (0, 1000] Hz"};
        period_ = std::chrono::duration<double>{1.0 / rate};
        const auto imu_rpy = vector_parameter("imu_rpy");
        base_from_sensor_ = rotation(imu_rpy);
        yaw_xyz_ = vector_parameter("yaw_xyz");
        yaw_scale_ = get_parameter("yaw_angle_scale").as_double();
        yaw_offset_ = get_parameter("yaw_angle_offset").as_double();
        if (!std::isfinite(yaw_scale_) || !std::isfinite(yaw_offset_))
            throw std::runtime_error{"yaw angle scale and offset must be finite"};
        const auto stamp = now();
        static_tf_.sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{
            transform("base_link", "imu_link", vector_parameter("imu_xyz"), base_from_sensor_, stamp),
            transform("yaw_link", "muzzle_link", vector_parameter("muzzle_xyz"),
                      rotation(vector_parameter("muzzle_rpy")), stamp)});
    }

    void publish_imu_tf() {
        const auto steady_now = std::chrono::steady_clock::now();
        if (steady_now < next_publish_)
            return;
        next_publish_ = steady_now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(period_);
        const auto stamp = now();
        // Encoder zero/scale defines the actual yaw joint angle, not the software-limit origin.
        if (*motor_feedback_ready_)
            dynamic_tf_.sendTransform(transform("base_link", "yaw_link", yaw_xyz_,
                Eigen::Quaterniond{Eigen::AngleAxisd{yaw_motor_.angle() * yaw_scale_ + yaw_offset_,
                                                    Eigen::Vector3d::UnitZ()}}, stamp));
        // Copy callback-thread sensor state, then release the lock before ROS publishing.
        std::optional<ImuState> sample;
        {
            const std::scoped_lock lock{imu_mutex_};
            sample = imu_state_;
        }
        if (!sample)
            return;
        const auto& state = *sample;
        // Do not repeatedly stamp stale sensor data as new measurements.
        if (last_sample_ && *last_sample_ == state.snapshot.timestamp)
            return;
        last_sample_ = state.snapshot.timestamp;
        sensor_msgs::msg::Imu message;
        // This is host receipt/publication time; board-clock ticks are not ROS epoch time.
        message.header.stamp = stamp;
        message.header.frame_id = "imu_link";
        const auto& q = state.snapshot.orientation;
        message.orientation.w = q.w(); message.orientation.x = q.x();
        message.orientation.y = q.y(); message.orientation.z = q.z();
        message.angular_velocity.x = state.snapshot.gyro_body.x();
        message.angular_velocity.y = state.snapshot.gyro_body.y();
        message.angular_velocity.z = state.snapshot.gyro_body.z();
        message.linear_acceleration.x = state.acceleration.x();
        message.linear_acceleration.y = state.acceleration.y();
        message.linear_acceleration.z = state.acceleration.z();
        // Zero covariance means unknown; no uncalibrated accuracy is claimed.
        imu_publisher_->publish(message);
        // IMU supplies attitude only: no translation estimate and no absolute yaw reference.
        dynamic_tf_.sendTransform(transform("odom_imu", "base_link", Eigen::Vector3d::Zero(),
                                           q * base_from_sensor_.conjugate(), stamp));
    }

    Eigen::Vector3d vector_parameter(const char* name) {
        const auto values = get_parameter(name).as_double_array();
        if (values.size() != 3 || !std::isfinite(values[0])
            || !std::isfinite(values[1]) || !std::isfinite(values[2]))
            throw std::runtime_error{std::string{name} + " must contain three finite values"};
        return {values[0], values[1], values[2]};
    }
    static Eigen::Quaterniond rotation(const Eigen::Vector3d& rpy) {
        return Eigen::AngleAxisd{rpy.z(), Eigen::Vector3d::UnitZ()}
             * Eigen::AngleAxisd{rpy.y(), Eigen::Vector3d::UnitY()}
             * Eigen::AngleAxisd{rpy.x(), Eigen::Vector3d::UnitX()};
    }
    static geometry_msgs::msg::TransformStamped transform(
        const char* parent, const char* child, const Eigen::Vector3d& xyz,
        const Eigen::Quaterniond& q, const rclcpp::Time& stamp) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = parent;
        tf.child_frame_id = child;
        tf.transform.translation.x = xyz.x(); tf.transform.translation.y = xyz.y();
        tf.transform.translation.z = xyz.z();
        tf.transform.rotation.w = q.w(); tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y(); tf.transform.rotation.z = q.z();
        return tf;
    }
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
    // CBoard dispatches sensor callbacks on its receive thread; update() reads a locked copy.
    device::Bmi088Ekf imu_;
    device::BoardClockLifter imu_clock_;
    Eigen::Vector3d latest_acceleration_ = Eigen::Vector3d::Zero();
    std::mutex imu_mutex_;
    struct ImuState {
        rmcs_msgs::ImuSnapshot snapshot;
        Eigen::Vector3d acceleration; // m/s^2 in the physical BMI088 sensor frame
    };
    std::optional<ImuState> imu_state_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    tf2_ros::TransformBroadcaster dynamic_tf_;
    tf2_ros::StaticTransformBroadcaster static_tf_;
    Eigen::Quaterniond base_from_sensor_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d yaw_xyz_ = Eigen::Vector3d::Zero();
    double yaw_scale_ = 1.0, yaw_offset_ = 0.0;
    std::chrono::duration<double> period_{0.01};
    std::chrono::steady_clock::time_point next_publish_{};
    std::optional<rmcs_msgs::BoardClock::time_point> last_sample_;
    std::atomic<unsigned> motor_feedback_mask_{0};
    OutputInterface<bool> motor_feedback_ready_;
    OutputInterface<bool> yaw_motor_feedback_ready_, distance_motor_feedback_ready_;
    OutputInterface<bool> trigger_pair_feedback_ready_;
    std::array<device::CBoardServo, 4> servos_;
    std::array<device::CBoardButton, 3> buttons_;
    device::DjiMotor trigger_motors_[2]{
        {*this, *command_component_, "/trigger/left_motor", device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 1}.set_encoder_zero_point(static_cast<int>(get_parameter("left_motor_zero_point").as_int())).enable_multi_turn_angle()},
        {*this, *command_component_, "/trigger/right_motor", device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 2}.set_encoder_zero_point(static_cast<int>(get_parameter("right_motor_zero_point").as_int())).enable_multi_turn_angle()},
    };
    device::DjiMotor yaw_motor_{*this, *command_component_, "/yaw_motor", device::DjiMotor::Config{device::DjiMotor::Type::kM2006, 3}.set_encoder_zero_point(static_cast<int>(get_parameter("yaw_motor_zero_point").as_int())).enable_multi_turn_angle()};
    device::DjiMotor distance_motor_{*this, *command_component_, "/distance_motor", device::DjiMotor::Config{device::DjiMotor::Type::kM2006, 4}.set_encoder_zero_point(static_cast<int>(get_parameter("distance_motor_zero_point").as_int())).enable_multi_turn_angle()};
    InputInterface<double> servo_angles_[4];
    static constexpr std::uint32_t kServoUpdateDivider = 20;
    std::uint32_t servo_update_ticks_ = 0;
    OutputInterface<bool> fire_button_output_;
    OutputInterface<bool> reload_button_output_;
    OutputInterface<bool> reload_cancel_button_output_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::DartLauncherHardware, rmcs_executor::Component)
