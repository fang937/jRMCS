#include <array>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <optional>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <eigen3/Eigen/Core>

namespace rmcs_core::controller::shooting {

class DartLauncherController : public rmcs_executor::Component, public rclcpp::Node {
public:
    DartLauncherController()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/joystick/right", joystick_right_);
        register_input("/dart/button/fire", fire_button_);
        register_input("/dart/button/reload", reload_button_);
        register_input("/dart/button/reload_cancel", reload_cancel_button_);
        for (std::size_t i = 0; i < servo_angles_.size(); ++i)
            register_output("/dart/servo/" + std::to_string(i + 1) + "/angle", servo_angles_[i], 90.0);
        register_input("/trigger/left_motor/angle", trigger_angle_[0]);
        register_input("/trigger/right_motor/angle", trigger_angle_[1]);
        register_input("/yaw_motor/angle", yaw_angle_);
        register_input("/distance_motor/angle", distance_angle_);
        register_output("/trigger/left_motor/control_angle", trigger_target_[0]);
        register_output("/trigger/right_motor/control_angle", trigger_target_[1]);
        register_output("/yaw_motor/control_angle", yaw_target_);
        register_output("/distance_motor/control_angle", distance_target_);

        fire_angles_ = read_angles("fire_angles", {60.0, 90.0, 90.0, 90.0});
        safe_angles_ = read_angles("safe_angles", {90.0, 90.0, 90.0, 90.0});
        fire_return_angles_ = read_angles("fire_return_angles", safe_angles_);
        reload_angles_[0] = read_angles("reload1_angles", safe_angles_);
        reload_angles_[1] = read_angles("reload2_angles", safe_angles_);
        reload_angles_[2] = read_angles("reload3_angles", safe_angles_);
        reload_return_angles_[0] = read_angles("reload1_return_angles", safe_angles_);
        reload_return_angles_[1] = read_angles("reload2_return_angles", safe_angles_);
        reload_return_angles_[2] = read_angles("reload3_return_angles", safe_angles_);
        fire_hold_ticks_ = std::max(1, static_cast<int>(get_parameter("fire_hold_ticks").as_int()));
        trigger_fire_stroke_ = get_parameter("trigger_fire_stroke").as_double();
        reload_interval_ticks_[0] = std::max(1, static_cast<int>(get_parameter("reload1_interval_ticks").as_int()));
        reload_interval_ticks_[1] = std::max(1, static_cast<int>(get_parameter("reload2_interval_ticks").as_int()));
        reload_interval_ticks_[2] = std::max(1, static_cast<int>(get_parameter("reload3_interval_ticks").as_int()));
        yaw_velocity_limit_ = get_parameter("yaw_motor_velocity_limit").as_double();
        distance_velocity_limit_ = get_parameter("distance_motor_velocity_limit").as_double();
        trigger_adjust_sensitivity_ = get_parameter("trigger_adjust_sensitivity").as_double();
        joystick_deadzone_ = get_parameter("joystick_deadzone").as_double();
        if (has_parameter("switch_debounce_ticks"))
            switch_debounce_ticks_ = std::max(1, static_cast<int>(get_parameter("switch_debounce_ticks").as_int()));
        set_angles(safe_angles_);
    }

    void update() override {
        if (!switch_left_.ready() || !switch_right_.ready()) {
            set_angles(safe_angles_);
            state_ = State::kSafe;
            reset_switches();
            return;
        }
        const auto left = *switch_left_;
        const auto right = *switch_right_;
        if (left == rmcs_msgs::Switch::UNKNOWN || right == rmcs_msgs::Switch::UNKNOWN
            || (left == rmcs_msgs::Switch::DOWN && right == rmcs_msgs::Switch::DOWN)) {
            state_ = State::kSafe;
            reset_switches();
            ticks_ = 0;
            set_angles(safe_angles_);
            hold_motor_positions();
            last_fire_ = fire_button_.ready() && *fire_button_;
            last_reload_ = reload_button_.ready() && *reload_button_;
            return;
        }

        if (!switches_initialized_) {
            left_switch_ = {left, left, 0};
            right_switch_ = {right, right, 0};
            switches_initialized_ = true;
        }
        const auto previous_left = left_switch_.stable;
        const auto previous_right = right_switch_.stable;
        update_switch(left, left_switch_);
        update_switch(right, right_switch_);
        const bool left_middle = previous_left != left_switch_.stable
            && left_switch_.stable == rmcs_msgs::Switch::MIDDLE;
        const bool right_middle = previous_right != right_switch_.stable
            && right_switch_.stable == rmcs_msgs::Switch::MIDDLE;
        const bool right_down = previous_right != right_switch_.stable
            && right_switch_.stable == rmcs_msgs::Switch::DOWN;
        if (safety_locked_ && left != rmcs_msgs::Switch::DOWN
            && right != rmcs_msgs::Switch::DOWN) {
            safety_locked_ = false;
            RCLCPP_INFO(get_logger(), "Safety lock released: both switches leave DOWN");
        }
        if (safety_locked_)
            return;

        if (!trigger_initialized_ && trigger_angle_[0].ready() && trigger_angle_[1].ready()) {
            trigger_target_value_[0] = *trigger_angle_[0];
            trigger_target_value_[1] = *trigger_angle_[1];
            trigger_initialized_ = true;
        }
        if (yaw_angle_.ready() && distance_angle_.ready() && !aim_initialized_) {
            yaw_target_value_ = *yaw_angle_;
            distance_target_value_ = *distance_angle_;
            aim_initialized_ = true;
        }
        if (aim_initialized_) {
            if (joystick_right_.ready()) {
                const auto right_stick = *joystick_right_;
                yaw_target_value_ += right_stick.x() * yaw_velocity_limit_ * 0.001;
                distance_target_value_ += right_stick.y() * distance_velocity_limit_ * 0.001;
                RCLCPP_INFO_THROTTLE(
                    get_logger(), *get_clock(), 1000,
                    "Right joystick: x=%.3f y=%.3f -> yaw_target=%.3f distance_target=%.3f",
                    right_stick.x(), right_stick.y(), yaw_target_value_, distance_target_value_);
            }
            *yaw_target_ = yaw_target_value_;
            *distance_target_ = distance_target_value_;
        } else if (joystick_right_.ready()) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Right joystick is ready but motor angle feedback is not initialized; yaw_ready=%s distance_ready=%s",
                yaw_angle_.ready() ? "yes" : "no", distance_angle_.ready() ? "yes" : "no");
        }
        if (trigger_initialized_ && state_ == State::kReady && joystick_left_.ready()) {
            const auto stick_y = std::abs((*joystick_left_).y()) > joystick_deadzone_
                ? (*joystick_left_).y() : 0.0;
            trigger_target_value_[0] -= stick_y * trigger_adjust_sensitivity_;
            trigger_target_value_[1] += stick_y * trigger_adjust_sensitivity_;
            *trigger_target_[0] = trigger_target_value_[0];
            *trigger_target_[1] = trigger_target_value_[1];
        }

        const bool fire = fire_button_.ready() && *fire_button_;
        const bool reload = reload_button_.ready() && *reload_button_;
        const bool cancel = reload_cancel_button_.ready() && *reload_cancel_button_;
        const bool fire_edge = (fire && !last_fire_) || left_middle;
        const bool reload_edge = (reload && !last_reload_) || right_middle;
        last_fire_ = fire;
        last_reload_ = reload;

        if (right_down && left != rmcs_msgs::Switch::DOWN && state_ != State::kFiring) {
            if (reload_count_ > 0) {
                --reload_count_;
                reload_stage_ = static_cast<int>(std::min<std::size_t>(reload_count_, 2));
                apply_reload_angles(reload_angles_[reload_stage_]);
                ticks_ = 0;
                state_ = State::kReady;
                RCLCPP_INFO(get_logger(), "Right switch DOWN: rollback reload to stage %d", reload_stage_ + 1);
            } else {
                RCLCPP_WARN(get_logger(), "Reload rollback ignored: no previous stage");
            }
            return;
        }
        switch (state_) {
        case State::kSafe:
        case State::kReady:
            state_ = State::kReady;
            if (fire_edge && right != rmcs_msgs::Switch::DOWN) {
                RCLCPP_INFO(get_logger(), "FIRE requested: left switch MIDDLE or fire button");
                state_ = State::kFiring;
                ticks_ = fire_hold_ticks_;
                set_angles(fire_angles_);
                trigger_return_target_ = trigger_target_value_;
                trigger_target_value_[0] -= trigger_fire_stroke_;
                trigger_target_value_[1] += trigger_fire_stroke_;
                *trigger_target_[0] = trigger_target_value_[0];
                *trigger_target_[1] = trigger_target_value_[1];
            } else if (reload_edge && left != rmcs_msgs::Switch::DOWN) {
                state_ = State::kReloading;
                reload_stage_ = static_cast<int>(std::min<std::size_t>(reload_count_, 2));
                ++reload_count_;
                ticks_ = reload_interval_ticks_[reload_stage_];
                apply_reload_angles(reload_angles_[reload_stage_]);
                RCLCPP_INFO(get_logger(), "RELOAD requested: right switch MIDDLE or reload button, stage=%d", reload_stage_ + 1);
            }
            break;
        case State::kFiring:
            if (--ticks_ <= 0) {
                state_ = State::kReady;
                set_angles(fire_return_angles_);
                restore_trigger_target();
            }
            break;
        case State::kReloading:
            if (cancel) {
                state_ = State::kReady;
                set_angles(safe_angles_);
                restore_trigger_target();
            } else if (--ticks_ <= 0) {
                apply_reload_angles(reload_return_angles_[reload_stage_]);
                state_ = State::kReady;
            }
            break;
        }
    }

private:
    struct SwitchState {
        rmcs_msgs::Switch candidate = rmcs_msgs::Switch::UNKNOWN;
        rmcs_msgs::Switch stable = rmcs_msgs::Switch::UNKNOWN;
        int ticks = 0;
    };
    void reset_switches() {
        if (!safety_locked_)
            RCLCPP_INFO(get_logger(), "Remote unavailable or both switches DOWN: safe lock");
        safety_locked_ = true;
        switches_initialized_ = false;
    }
    void update_switch(rmcs_msgs::Switch value, SwitchState& state) {
        if (state.candidate != value) {
            state.candidate = value;
            state.ticks = 0;
        }
        if (state.ticks < switch_debounce_ticks_)
            ++state.ticks;
        if (state.ticks >= switch_debounce_ticks_)
            state.stable = state.candidate;
    }
    std::array<double, 4> current_angles() const {
        std::array<double, 4> angles{};
        for (std::size_t i = 0; i < angles.size(); ++i)
            angles[i] = last_servo_angles_[i].value_or(safe_angles_[i]);
        return angles;
    }
    void apply_reload_angles(const std::array<double, 4>& targets) {
        auto angles = current_angles();
        angles[1] = targets[1];
        angles[2] = targets[2];
        if (reload_stage_ > 0)
            angles[3] = targets[3];
        set_angles(angles);
    }
    enum class State { kSafe, kReady, kFiring, kReloading };
    std::array<double, 4> read_angles(const char* name, std::array<double, 4> fallback) {
        if (!has_parameter(name))
            return fallback;
        const auto values = get_parameter(name).as_double_array();
        if (values.size() != fallback.size())
            throw std::runtime_error{std::string{name} + " must contain four angles"};
        std::copy(values.begin(), values.end(), fallback.begin());
        return fallback;
    }
    void set_angles(const std::array<double, 4>& angles) {
        bool changed = false;
        for (std::size_t i = 0; i < angles.size(); ++i) {
            changed |= !last_servo_angles_[i].has_value() || *last_servo_angles_[i] != angles[i];
            *servo_angles_[i] = angles[i];
            last_servo_angles_[i] = angles[i];
        }
        if (changed)
            RCLCPP_INFO(
                get_logger(), "Servo targets updated: [%.1f, %.1f, %.1f, %.1f]",
                angles[0], angles[1], angles[2], angles[3]);
    }
    void hold_motor_positions() {
        if (trigger_angle_[0].ready() && trigger_angle_[1].ready()) {
            trigger_target_value_[0] = *trigger_angle_[0];
            trigger_target_value_[1] = *trigger_angle_[1];
            *trigger_target_[0] = trigger_target_value_[0];
            *trigger_target_[1] = trigger_target_value_[1];
        }
        if (yaw_angle_.ready() && distance_angle_.ready()) {
            yaw_target_value_ = *yaw_angle_;
            distance_target_value_ = *distance_angle_;
            *yaw_target_ = yaw_target_value_;
            *distance_target_ = distance_target_value_;
        }
    }
    void restore_trigger_target() {
        trigger_target_value_ = trigger_return_target_;
        *trigger_target_[0] = trigger_target_value_[0];
        *trigger_target_[1] = trigger_target_value_[1];
    }

    InputInterface<rmcs_msgs::Switch> switch_left_, switch_right_;
    InputInterface<Eigen::Vector2d> joystick_left_, joystick_right_;
    InputInterface<bool> fire_button_, reload_button_, reload_cancel_button_;
    std::array<OutputInterface<double>, 4> servo_angles_;
    std::array<std::optional<double>, 4> last_servo_angles_;
    InputInterface<double> trigger_angle_[2], yaw_angle_, distance_angle_;
    OutputInterface<double> trigger_target_[2], yaw_target_, distance_target_;
    State state_ = State::kSafe;
    std::array<double, 4> safe_angles_{90.0, 90.0, 90.0, 90.0};
    std::array<std::array<double, 4>, 3> reload_angles_{};
    std::array<std::array<double, 4>, 3> reload_return_angles_{};
    std::array<double, 4> fire_return_angles_{};
    std::array<double, 4> fire_angles_{60.0, 90.0, 90.0, 90.0};
    int fire_hold_ticks_ = 500;
    std::array<int, 3> reload_interval_ticks_{2000, 2000, 2000};
    int ticks_ = 0;
    int reload_stage_ = 0;
    std::size_t reload_count_ = 0;
    SwitchState left_switch_, right_switch_;
    int switch_debounce_ticks_ = 20;
    bool switches_initialized_ = false;
    bool safety_locked_ = true;
    bool last_fire_ = false;
    bool last_reload_ = false;
    bool trigger_initialized_ = false;
    bool aim_initialized_ = false;
    std::array<double, 2> trigger_target_value_{};
    std::array<double, 2> trigger_return_target_{};
    double yaw_target_value_ = 0.0;
    double distance_target_value_ = 0.0;
    double yaw_velocity_limit_ = 10.0;
    double distance_velocity_limit_ = 10.0;
    double trigger_fire_stroke_ = 0.5;
    double trigger_adjust_sensitivity_ = 0.002;
    double joystick_deadzone_ = 0.08;
};

} // namespace rmcs_core::controller::shooting

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::shooting::DartLauncherController, rmcs_executor::Component)
