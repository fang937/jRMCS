#include <array>
#include <algorithm>
#include <cmath>
#include <numbers>
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
        register_input("/dart/motors/feedback_ready", motor_feedback_ready_);
        register_input("/dart/button/fire", fire_button_);
        register_input("/dart/button/reload", reload_button_);
        register_input("/dart/button/reload_cancel", reload_cancel_button_);
        // 原扳机舵机 1 已改为 M2006，剩余舵机保持物理编号 2/3/4。
        for (std::size_t i = 0; i < servo_angles_.size(); ++i)
            register_output(
                "/dart/servo/" + std::to_string(i + 2) + "/angle", servo_angles_[i], 90.0);
        register_input("/trigger/left_motor/angle", trigger_angle_[0]);
        register_input("/trigger/right_motor/angle", trigger_angle_[1]);
        register_input("/trigger/motor/angle", trigger_motor_angle_);
        register_input("/yaw_motor/angle", yaw_angle_);
        register_input("/distance_motor/angle", distance_angle_);
        register_output("/trigger/left_motor/control_angle", trigger_target_[0]);
        register_output("/trigger/right_motor/control_angle", trigger_target_[1]);
        register_output("/trigger/motor/control_angle", trigger_motor_target_);
        register_output("/yaw_motor/control_angle", yaw_target_);
        register_output("/distance_motor/control_angle", distance_target_);

        fire_angles_ = read_angles("fire_angles", {90.0, 90.0, 90.0});
        safe_angles_ = read_angles("safe_angles", {90.0, 90.0, 90.0});
        fire_return_angles_ = read_angles("fire_return_angles", safe_angles_);
        reload_angles_[0] = read_angles("reload1_angles", safe_angles_);
        reload_angles_[1] = read_angles("reload2_angles", safe_angles_);
        reload_angles_[2] = read_angles("reload3_angles", safe_angles_);
        reload_return_angles_[0] = read_angles("reload1_return_angles", safe_angles_);
        reload_return_angles_[1] = read_angles("reload2_return_angles", safe_angles_);
        reload_return_angles_[2] = read_angles("reload3_return_angles", safe_angles_);
        fire_hold_ticks_ = std::max(1, static_cast<int>(get_parameter("fire_hold_ticks").as_int()));
        reload_interval_ticks_[0] = std::max(1, static_cast<int>(get_parameter("reload1_interval_ticks").as_int()));
        reload_interval_ticks_[1] = std::max(1, static_cast<int>(get_parameter("reload2_interval_ticks").as_int()));
        reload_interval_ticks_[2] = std::max(1, static_cast<int>(get_parameter("reload3_interval_ticks").as_int()));
        yaw_velocity_limit_ = get_parameter("yaw_motor_velocity_limit").as_double();
        distance_velocity_limit_ = get_parameter("distance_motor_velocity_limit").as_double();
        trigger_adjust_sensitivity_ = get_parameter("trigger_adjust_sensitivity").as_double();
        joystick_deadzone_ = get_parameter("joystick_deadzone").as_double();
        if (has_parameter("switch_debounce_ticks"))
            switch_debounce_ticks_ = std::max(1, static_cast<int>(get_parameter("switch_debounce_ticks").as_int()));

        // 扳机 M2006 的发射/返回目标，单位：圈 + 附加角度。
        trigger_fire_position_ = read_trigger_position("trigger_fire");
        trigger_return_position_ = read_trigger_position("trigger_return");
        trigger_min_turns_ = get_parameter("trigger_motor_position_min_turns").as_double();
        trigger_max_turns_ = get_parameter("trigger_motor_position_max_turns").as_double();
        if (!std::isfinite(trigger_min_turns_) || !std::isfinite(trigger_max_turns_)
            || trigger_min_turns_ > 0.0 || trigger_max_turns_ < 0.0)
            throw std::runtime_error{
                "trigger_motor_position_min_turns/max_turns must be finite and contain zero"};

        // Limits are output-shaft radians relative to the first valid feedback at startup.
        const std::array<std::string, 4> names{"left_motor", "right_motor", "yaw_motor", "distance_motor"};
        for (std::size_t i = 0; i < names.size(); ++i) {
            motor_min_[i] = get_parameter(names[i] + "_position_min").as_double();
            motor_max_[i] = get_parameter(names[i] + "_position_max").as_double();
            if (!std::isfinite(motor_min_[i]) || !std::isfinite(motor_max_[i])
                || motor_min_[i] > 0.0 || motor_max_[i] < 0.0)
                throw std::runtime_error{names[i] + " position limits must be finite and contain zero"};
        }
        set_angles(safe_angles_);
    }

    void update() override {
        // Interface readiness alone does not mean a CAN feedback frame has arrived.
        if (motor_feedback_ready_.ready() && *motor_feedback_ready_ && !motor_positions_initialized_
            && trigger_angle_[0].ready() && trigger_angle_[1].ready()
            && trigger_motor_angle_.ready() && yaw_angle_.ready() && distance_angle_.ready()) {
            motor_origins_ = {*trigger_angle_[0], *trigger_angle_[1], *yaw_angle_, *distance_angle_};
            trigger_target_value_ = {motor_origins_[0], motor_origins_[1]};
            trigger_motor_origin_ = *trigger_motor_angle_;
            trigger_motor_target_value_ = trigger_motor_origin_;
            trigger_motor_min_ =
                trigger_motor_origin_ + trigger_min_turns_ * 2.0 * std::numbers::pi;
            trigger_motor_max_ =
                trigger_motor_origin_ + trigger_max_turns_ * 2.0 * std::numbers::pi;
            yaw_target_value_ = motor_origins_[2];
            distance_target_value_ = motor_origins_[3];
            motor_positions_initialized_ = true;
            publish_motor_targets();
        }
        if (!switch_left_.ready() || !switch_right_.ready()) {
            set_angles(safe_angles_);
            state_ = State::kSafe;
            reset_switches();
            hold_motor_positions();
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

        if (motor_positions_initialized_) {
            if (joystick_right_.ready()) {
                const auto& stick = *joystick_right_;
                // Integrate joystick input into position commands at the configured 1 kHz update rate.
                yaw_target_value_ += deadband(stick.x()) * yaw_velocity_limit_ * 0.001;
                distance_target_value_ += deadband(stick.y()) * distance_velocity_limit_ * 0.001;
            }
            // 左拨杆只调整新增的扳机 M2006，不再修改左右扳机 M3508 的目标。
            if (state_ == State::kReady && joystick_left_.ready()) {
                const auto stick_y = deadband((*joystick_left_).y());
                trigger_motor_target_value_ -= stick_y * trigger_adjust_sensitivity_;
            }
            publish_motor_targets();
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
                set_trigger_target(trigger_fire_position_);
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
                set_trigger_target(trigger_return_position_);
            }
            break;
        case State::kReloading:
            if (cancel) {
                state_ = State::kReady;
                set_angles(safe_angles_);
            } else if (--ticks_ <= 0) {
                apply_reload_angles(reload_return_angles_[reload_stage_]);
                state_ = State::kReady;
            }
            break;
        }
    }

private:
    struct TriggerPosition {
        double turns = 0.0;
        double angle_deg = 0.0;
    };
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
    std::array<double, 3> current_angles() const {
        std::array<double, 3> angles{};
        for (std::size_t i = 0; i < angles.size(); ++i)
            angles[i] = last_servo_angles_[i].value_or(safe_angles_[i]);
        return angles;
    }
    void apply_reload_angles(const std::array<double, 3>& targets) {
        auto angles = current_angles();
        angles[0] = targets[0];
        angles[1] = targets[1];
        if (reload_stage_ > 0)
            angles[2] = targets[2];
        set_angles(angles);
    }
    enum class State { kSafe, kReady, kFiring, kReloading };
    std::array<double, 3> read_angles(const char* name, std::array<double, 3> fallback) {
        if (!has_parameter(name))
            return fallback;
        const auto values = get_parameter(name).as_double_array();
        if (values.size() != fallback.size())
            throw std::runtime_error{std::string{name} + " must contain three angles"};
        std::copy(values.begin(), values.end(), fallback.begin());
        return fallback;
    }
    TriggerPosition read_trigger_position(const std::string& prefix) {
        TriggerPosition position;
        position.turns = get_parameter(prefix + "_turns").as_double();
        position.angle_deg = get_parameter(prefix + "_angle_deg").as_double();
        if (!std::isfinite(position.turns) || !std::isfinite(position.angle_deg))
            throw std::runtime_error{prefix + " position must be finite"};
        return position;
    }
    void set_angles(const std::array<double, 3>& angles) {
        bool changed = false;
        for (std::size_t i = 0; i < angles.size(); ++i) {
            changed |= !last_servo_angles_[i].has_value() || *last_servo_angles_[i] != angles[i];
            *servo_angles_[i] = angles[i];
            last_servo_angles_[i] = angles[i];
        }
        if (changed)
            RCLCPP_INFO(
                get_logger(), "Servo targets updated: [%.1f, %.1f, %.1f]",
                angles[0], angles[1], angles[2]);
    }
    double deadband(double value) const {
        return std::abs(value) > joystick_deadzone_ ? value : 0.0;
    }
    void set_trigger_target(const TriggerPosition& position) {
        if (!motor_positions_initialized_)
            return;
        trigger_motor_target_value_ = trigger_motor_origin_
            + position.turns * 2.0 * std::numbers::pi
            + position.angle_deg * std::numbers::pi / 180.0;
        trigger_motor_target_value_ = limited_trigger_target(trigger_motor_target_value_);
        *trigger_motor_target_ = trigger_motor_target_value_;
        RCLCPP_INFO(
            get_logger(), "Trigger motor target: %.3f rad (turns=%.3f, angle=%.1f deg)",
            trigger_motor_target_value_, position.turns, position.angle_deg);
    }
    void publish_motor_targets() {
        // Clamp the accumulator too, so reversing the stick immediately leaves the limit.
        trigger_target_value_[0] = limited_target(trigger_target_value_[0], 0);
        trigger_target_value_[1] = limited_target(trigger_target_value_[1], 1);
        yaw_target_value_ = limited_target(yaw_target_value_, 2);
        distance_target_value_ = limited_target(distance_target_value_, 3);
        trigger_motor_target_value_ = limited_trigger_target(trigger_motor_target_value_);
        *trigger_target_[0] = trigger_target_value_[0];
        *trigger_target_[1] = trigger_target_value_[1];
        *trigger_motor_target_ = trigger_motor_target_value_;
        *yaw_target_ = yaw_target_value_;
        *distance_target_ = distance_target_value_;
    }
    double limited_target(double target, std::size_t index) const {
        if (!std::isfinite(target))
            target = motor_origins_[index];
        return std::clamp(target, motor_origins_[index] + motor_min_[index],
                          motor_origins_[index] + motor_max_[index]);
    }
    double limited_trigger_target(double target) const {
        if (!std::isfinite(target))
            target = trigger_motor_origin_;
        return std::clamp(target, trigger_motor_min_, trigger_motor_max_);
    }
    void hold_motor_positions() {
        if (!motor_positions_initialized_)
            return;
        trigger_target_value_ = {*trigger_angle_[0], *trigger_angle_[1]};
        trigger_motor_target_value_ = *trigger_motor_angle_;
        yaw_target_value_ = *yaw_angle_;
        distance_target_value_ = *distance_angle_;
        publish_motor_targets();
    }
    InputInterface<bool> motor_feedback_ready_;
    std::array<double, 4> motor_min_{}, motor_max_{}, motor_origins_{};
    InputInterface<rmcs_msgs::Switch> switch_left_, switch_right_;
    InputInterface<Eigen::Vector2d> joystick_left_, joystick_right_;
    InputInterface<bool> fire_button_, reload_button_, reload_cancel_button_;
    std::array<OutputInterface<double>, 3> servo_angles_;
    std::array<std::optional<double>, 3> last_servo_angles_;
    InputInterface<double> trigger_angle_[2], trigger_motor_angle_, yaw_angle_, distance_angle_;
    OutputInterface<double> trigger_target_[2], trigger_motor_target_, yaw_target_, distance_target_;
    State state_ = State::kSafe;
    std::array<double, 3> safe_angles_{90.0, 90.0, 90.0};
    std::array<std::array<double, 3>, 3> reload_angles_{};
    std::array<std::array<double, 3>, 3> reload_return_angles_{};
    std::array<double, 3> fire_return_angles_{};
    std::array<double, 3> fire_angles_{90.0, 90.0, 90.0};
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
    bool motor_positions_initialized_ = false;
    std::array<double, 2> trigger_target_value_{};
    double yaw_target_value_ = 0.0;
    double distance_target_value_ = 0.0;
    double yaw_velocity_limit_ = 10.0;
    double distance_velocity_limit_ = 10.0;
    double trigger_adjust_sensitivity_ = 0.002;
    double joystick_deadzone_ = 0.08;
    TriggerPosition trigger_fire_position_{}, trigger_return_position_{};
    double trigger_motor_origin_ = 0.0;
    double trigger_motor_target_value_ = 0.0;
    double trigger_motor_min_ = 0.0, trigger_motor_max_ = 0.0;
    double trigger_min_turns_ = 0.0, trigger_max_turns_ = 0.0;
};

} // namespace rmcs_core::controller::shooting

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::shooting::DartLauncherController, rmcs_executor::Component)
