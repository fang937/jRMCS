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
        register_input("/yaw_motor/feedback_ready", yaw_motor_feedback_ready_);
        register_input("/distance_motor/feedback_ready", distance_motor_feedback_ready_);
        register_input("/dart/trigger_pair/feedback_ready", trigger_pair_feedback_ready_);
        register_input("/trigger/motor/feedback_ready", trigger_motor_feedback_ready_);
        register_input("/trigger/motor/velocity", trigger_motor_velocity_);
        register_input("/trigger/motor/encoder_raw", trigger_motor_encoder_raw_);
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
        register_input("/yaw_motor/velocity", yaw_velocity_);
        register_input("/distance_motor/velocity", distance_velocity_);
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
        reload_interval_ticks_[0] = std::max(1, static_cast<int>(get_parameter("reload1_interval_ticks").as_int()));
        reload_interval_ticks_[1] = std::max(1, static_cast<int>(get_parameter("reload2_interval_ticks").as_int()));
        reload_interval_ticks_[2] = std::max(1, static_cast<int>(get_parameter("reload3_interval_ticks").as_int()));
        yaw_velocity_limit_ = double_parameter("yaw_motor_velocity_limit");
        distance_velocity_limit_ = double_parameter("distance_motor_velocity_limit");
        trigger_adjust_sensitivity_ = double_parameter("trigger_adjust_sensitivity");
        joystick_deadzone_ = double_parameter("joystick_deadzone");
        if (has_parameter("switch_debounce_ticks"))
            switch_debounce_ticks_ = std::max(1, static_cast<int>(get_parameter("switch_debounce_ticks").as_int()));

        // 三档位置和软件限位都使用减速箱输出轴角度，单位为度，相对启动原点。
        trigger_fire_position_ = read_trigger_position("trigger_fire");
        trigger_pre_fire_position_ = read_trigger_position("trigger_pre_fire");
        trigger_contact_position_ = read_trigger_position("trigger_contact");
        trigger_min_angle_deg_ = double_parameter("trigger_motor_position_min_deg");
        trigger_max_angle_deg_ = double_parameter("trigger_motor_position_max_deg");
        if (!std::isfinite(trigger_min_angle_deg_) || !std::isfinite(trigger_max_angle_deg_)
            || trigger_min_angle_deg_ > 0.0 || trigger_max_angle_deg_ < 0.0)
            throw std::runtime_error{
                "trigger_motor_position_min_deg/max_deg must be finite and contain zero"};
        // 位置目标斜坡速度：把阶跃目标拆成每 tick 的小步，避免满力矩撞击机械限位。
        trigger_pre_fire_velocity_limit_ = double_parameter("trigger_pre_fire_velocity_limit");
        trigger_contact_velocity_limit_ = double_parameter("trigger_contact_velocity_limit");
        trigger_fire_velocity_limit_ = double_parameter("trigger_fire_velocity_limit");
        for (const auto velocity : {trigger_pre_fire_velocity_limit_, trigger_contact_velocity_limit_,
                                    trigger_fire_velocity_limit_})
            if (!std::isfinite(velocity) || velocity <= 0.0)
                throw std::runtime_error{"trigger position velocity limits must be positive"};

        // Limits are output-shaft radians relative to the first valid feedback at startup.
        const std::array<std::string, 4> names{"left_motor", "right_motor", "yaw_motor", "distance_motor"};
        for (std::size_t i = 0; i < names.size(); ++i) {
            motor_min_[i] = double_parameter(names[i] + "_position_min");
            motor_max_[i] = double_parameter(names[i] + "_position_max");
            if (!std::isfinite(motor_min_[i]) || !std::isfinite(motor_max_[i])
                || motor_min_[i] > 0.0 || motor_max_[i] < 0.0)
                throw std::runtime_error{names[i] + " position limits must be finite and contain zero"};
        }
        set_angles(safe_angles_);
    }

    void update() override {
        std::string action_name;
        if (state_ == State::kFiring) {
            action_name = "FIRING";
        } else if (state_ == State::kReloading) {
            action_name = "RELOADING (stage " + std::to_string(reload_stage_ + 1) + "/3)";
        } else {
            action_name = "IDLE";
        }
        const char* position_state = "UNKNOWN";
        if (left_switch_.stable == rmcs_msgs::Switch::UP)
            position_state = "PRE_FIRE";
        else if (left_switch_.stable == rmcs_msgs::Switch::MIDDLE)
            position_state = "CONTACT";
        else if (left_switch_.stable == rmcs_msgs::Switch::DOWN)
            position_state = "FIRE";
        const double nan = std::nan("");
        const double actual_angle_deg = trigger_position_initialized_ && trigger_motor_angle_.ready()
            ? (*trigger_motor_angle_ - trigger_motor_origin_) * 180.0 / std::numbers::pi
            : nan;
        const double ramp_target_deg = trigger_position_initialized_
            ? (trigger_motor_target_value_ - trigger_motor_origin_) * 180.0 / std::numbers::pi
            : nan;
        const double selected_angle_deg = trigger_position_initialized_
            ? (trigger_motor_command_value_ - trigger_motor_origin_) * 180.0 / std::numbers::pi
            : nan;
        const double motor_speed_deg_s = trigger_motor_velocity_.ready()
            ? *trigger_motor_velocity_ * 180.0 / std::numbers::pi
            : nan;
        const double slew_limit_deg_s = current_trigger_velocity_limit() * 180.0 / std::numbers::pi;
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 20,
            "Trigger state=%s action=%s encoder=%d/8191 angle=%.2f deg ramp_target=%.2f deg selected=%.2f deg motor_speed=%.2f deg/s target_slew=%.2f deg/s (limit %.2f deg/s)",
            position_state, action_name.c_str(),
            trigger_motor_encoder_raw_.ready() ? *trigger_motor_encoder_raw_ : -1, actual_angle_deg, ramp_target_deg,
            selected_angle_deg, motor_speed_deg_s, trigger_motor_target_slew_deg_s_,
            slew_limit_deg_s);
        const double yaw_angle_deg = yaw_position_initialized_ && yaw_angle_.ready()
            ? (*yaw_angle_ - motor_origins_[2]) * 180.0 / std::numbers::pi
            : nan;
        const double distance_angle_deg = distance_position_initialized_ && distance_angle_.ready()
            ? (*distance_angle_ - motor_origins_[3]) * 180.0 / std::numbers::pi
            : nan;
        const double right_x = joystick_right_.ready() ? (*joystick_right_).x() : nan;
        const double right_y = joystick_right_.ready() ? (*joystick_right_).y() : nan;
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 20,
            "Motor 203 yaw: stick_x=%.2f angle=%.2f deg target=%.2f deg speed=%.2f deg/s | "
            "Motor 204 distance: stick_y=%.2f angle=%.2f deg target=%.2f deg speed=%.2f deg/s | safety_locked=%s",
            right_x, yaw_angle_deg,
            yaw_position_initialized_
                ? (yaw_target_value_ - motor_origins_[2]) * 180.0 / std::numbers::pi : nan,
            yaw_velocity_.ready() ? *yaw_velocity_ * 180.0 / std::numbers::pi : nan,
            right_y, distance_angle_deg,
            distance_position_initialized_
                ? (distance_target_value_ - motor_origins_[3]) * 180.0 / std::numbers::pi : nan,
            distance_velocity_.ready() ? *distance_velocity_ * 180.0 / std::numbers::pi : nan,
            safety_locked_ ? "yes" : "no");

        // 初始化前让输出目标跟随实测角度。输出接口默认值是 0，如果此时电机反馈已经
        // 是非零角度，直接按 0 目标闭环会把扳机上电就拉向零位（表现为上电抖动）。
        if (trigger_pair_feedback_ready_.ready() && *trigger_pair_feedback_ready_
            && !trigger_pair_initialized_ && trigger_angle_[0].ready() && trigger_angle_[1].ready()) {
            motor_origins_[0] = *trigger_angle_[0];
            motor_origins_[1] = *trigger_angle_[1];
            trigger_target_value_ = {motor_origins_[0], motor_origins_[1]};
            trigger_pair_initialized_ = true;
        }
        if (yaw_motor_feedback_ready_.ready() && *yaw_motor_feedback_ready_
            && !yaw_position_initialized_ && yaw_angle_.ready()) {
            motor_origins_[2] = *yaw_angle_;
            yaw_target_value_ = motor_origins_[2];
            yaw_position_initialized_ = true;
        }
        if (distance_motor_feedback_ready_.ready() && *distance_motor_feedback_ready_
            && !distance_position_initialized_ && distance_angle_.ready()) {
            motor_origins_[3] = *distance_angle_;
            distance_target_value_ = motor_origins_[3];
            distance_position_initialized_ = true;
        }
        if (trigger_motor_feedback_ready_.ready() && *trigger_motor_feedback_ready_
            && trigger_motor_angle_.ready() && !trigger_position_initialized_) {
            trigger_motor_origin_ = *trigger_motor_angle_;
            trigger_motor_command_value_ = trigger_motor_origin_;
            trigger_motor_target_value_ = trigger_motor_origin_;
            trigger_motor_min_ =
                trigger_motor_origin_ + trigger_min_angle_deg_ * std::numbers::pi / 180.0;
            trigger_motor_max_ =
                trigger_motor_origin_ + trigger_max_angle_deg_ * std::numbers::pi / 180.0;
            trigger_position_initialized_ = true;
        }
        if (joystick_right_.ready()) {
            const auto& stick = *joystick_right_;
            if (yaw_position_initialized_)
                yaw_target_value_ += deadband(stick.x()) * yaw_velocity_limit_ * 0.001;
            if (distance_position_initialized_)
                distance_target_value_ += deadband(stick.y()) * distance_velocity_limit_ * 0.001;
        }
        if (trigger_pair_initialized_ && joystick_left_.ready()) {
            const auto stick_y = deadband((*joystick_left_).y());
            trigger_target_value_[0] -= stick_y * trigger_adjust_sensitivity_;
            trigger_target_value_[1] += stick_y * trigger_adjust_sensitivity_;
        }
        if (yaw_position_initialized_ || distance_position_initialized_ || trigger_pair_initialized_
            || trigger_position_initialized_)
            publish_motor_targets();
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
        const auto previous_right = right_switch_.stable;
        update_switch(left, left_switch_);
        update_switch(right, right_switch_);
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

        if (trigger_position_initialized_ && left_switch_.stable != rmcs_msgs::Switch::UNKNOWN) {
            if (left_switch_.stable == rmcs_msgs::Switch::UP) {
                trigger_fire_position_selected_ = false;
                trigger_contact_position_selected_ = false;
                set_trigger_target(trigger_pre_fire_position_);
            } else if (left_switch_.stable == rmcs_msgs::Switch::DOWN) {
                trigger_fire_position_selected_ = true;
                trigger_contact_position_selected_ = false;
                set_trigger_target(trigger_fire_position_);
            } else {
                trigger_fire_position_selected_ = false;
                trigger_contact_position_selected_ = true;
                set_trigger_target(trigger_contact_position_);
            }
        }

        const bool fire = fire_button_.ready() && *fire_button_;
        const bool reload = reload_button_.ready() && *reload_button_;
        const bool cancel = reload_cancel_button_.ready() && *reload_cancel_button_;
        const bool fire_edge = fire && !last_fire_;
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
                set_angles(fire_angles_);
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
            state_ = State::kReady;
            set_angles(fire_return_angles_);
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
        position.angle_deg = double_parameter(prefix + "_angle_deg");
        if (!std::isfinite(position.angle_deg))
            throw std::runtime_error{prefix + " position must be finite"};
        return position;
    }
    // YAML 里写成 0、1、-314 会被解析成 integer；这里统一转成 double，避免类型异常。
    double double_parameter(const std::string& name) {
        const auto parameter = get_parameter(name);
        if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            return static_cast<double>(parameter.as_int());
        return parameter.as_double();
    }
    void set_angles(const std::array<double, 3>& angles) {
        for (std::size_t i = 0; i < angles.size(); ++i) {
            *servo_angles_[i] = angles[i];
            last_servo_angles_[i] = angles[i];
        }
    }
    double deadband(double value) const {
        return std::abs(value) > joystick_deadzone_ ? value : 0.0;
    }
    void set_trigger_target(const TriggerPosition& position) {
        if (!trigger_position_initialized_)
            return;
        // 只更新目标指令，实际输出由 publish_motor_targets() 按斜率逼近，避免阶跃。
        trigger_motor_command_value_ =
            limited_trigger_target(trigger_position_value(position));
        RCLCPP_INFO(
            get_logger(), "Trigger motor selected angle: %.2f deg from startup origin",
            (trigger_motor_command_value_ - trigger_motor_origin_) * 180.0 / std::numbers::pi);
    }
    double trigger_position_value(const TriggerPosition& position) const {
        return trigger_motor_origin_ + position.angle_deg * std::numbers::pi / 180.0;
    }
    double current_trigger_velocity_limit() const {
        if (trigger_fire_position_selected_)
            return trigger_fire_velocity_limit_;
        if (trigger_contact_position_selected_)
            return trigger_contact_velocity_limit_;
        return trigger_pre_fire_velocity_limit_;
    }
    void publish_motor_targets() {
        // Clamp the accumulator too, so reversing the stick immediately leaves the limit.
        if (trigger_pair_initialized_) {
            trigger_target_value_[0] = limited_target(trigger_target_value_[0], 0);
            trigger_target_value_[1] = limited_target(trigger_target_value_[1], 1);
        }
        if (yaw_position_initialized_)
            yaw_target_value_ = limited_target(yaw_target_value_, 2);
        if (distance_position_initialized_)
            distance_target_value_ = limited_target(distance_target_value_, 3);
        // 1 kHz 更新：目标按配置的输出轴角速度斜坡靠近指令值。
        const double slew_rate = current_trigger_velocity_limit();
        const double max_step = slew_rate * 0.001;
        const double previous_target = trigger_motor_target_value_;
        trigger_motor_target_value_ += std::clamp(
            trigger_motor_command_value_ - trigger_motor_target_value_, -max_step, max_step);
        trigger_motor_target_value_ = limited_trigger_target(trigger_motor_target_value_);
        trigger_motor_target_slew_deg_s_ =
            (trigger_motor_target_value_ - previous_target) * 1000.0 * 180.0 / std::numbers::pi;
        if (trigger_pair_initialized_) {
            *trigger_target_[0] = trigger_target_value_[0];
            *trigger_target_[1] = trigger_target_value_[1];
        }
        if (trigger_position_initialized_)
            *trigger_motor_target_ = trigger_motor_target_value_;
        if (yaw_position_initialized_)
            *yaw_target_ = yaw_target_value_;
        if (distance_position_initialized_)
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
        if (!trigger_position_initialized_)
            return;
        // M2006 始终保持启动时锁存的绝对回位位置，回位途中进入安全状态也不改目标。
        trigger_fire_position_selected_ = false;
        trigger_contact_position_selected_ = false;
        trigger_motor_command_value_ =
            limited_trigger_target(trigger_position_value(trigger_pre_fire_position_));
        publish_motor_targets();
    }
    InputInterface<bool> motor_feedback_ready_;
    InputInterface<bool> yaw_motor_feedback_ready_, distance_motor_feedback_ready_;
    InputInterface<bool> trigger_pair_feedback_ready_;
    InputInterface<bool> trigger_motor_feedback_ready_;
    std::array<double, 4> motor_min_{}, motor_max_{}, motor_origins_{};
    InputInterface<rmcs_msgs::Switch> switch_left_, switch_right_;
    InputInterface<Eigen::Vector2d> joystick_left_, joystick_right_;
    InputInterface<bool> fire_button_, reload_button_, reload_cancel_button_;
    std::array<OutputInterface<double>, 3> servo_angles_;
    std::array<std::optional<double>, 3> last_servo_angles_;
    InputInterface<double> trigger_angle_[2], trigger_motor_angle_, trigger_motor_velocity_;
    InputInterface<int> trigger_motor_encoder_raw_;
    InputInterface<double> yaw_angle_, distance_angle_, yaw_velocity_, distance_velocity_;
    OutputInterface<double> trigger_target_[2], trigger_motor_target_, yaw_target_, distance_target_;
    State state_ = State::kSafe;
    std::array<double, 3> safe_angles_{90.0, 90.0, 90.0};
    std::array<std::array<double, 3>, 3> reload_angles_{};
    std::array<std::array<double, 3>, 3> reload_return_angles_{};
    std::array<double, 3> fire_return_angles_{};
    std::array<double, 3> fire_angles_{90.0, 90.0, 90.0};
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
    bool trigger_pair_initialized_ = false;
    bool yaw_position_initialized_ = false;
    bool distance_position_initialized_ = false;
    bool trigger_position_initialized_ = false;
    std::array<double, 2> trigger_target_value_{};
    double yaw_target_value_ = 0.0;
    double distance_target_value_ = 0.0;
    double yaw_velocity_limit_ = 10.0;
    double distance_velocity_limit_ = 10.0;
    double trigger_adjust_sensitivity_ = 0.002;
    double joystick_deadzone_ = 0.08;
    TriggerPosition trigger_fire_position_{}, trigger_pre_fire_position_{}, trigger_contact_position_{};
    double trigger_motor_origin_ = 0.0;
    double trigger_motor_command_value_ = 0.0;
    double trigger_motor_target_value_ = 0.0;
    double trigger_motor_target_slew_deg_s_ = 0.0;
    double trigger_pre_fire_velocity_limit_ = 1.0;
    double trigger_contact_velocity_limit_ = 1.0;
    double trigger_fire_velocity_limit_ = 5.0;
    bool trigger_fire_position_selected_ = false;
    bool trigger_contact_position_selected_ = false;
    double trigger_motor_min_ = 0.0, trigger_motor_max_ = 0.0;
    double trigger_min_angle_deg_ = 0.0, trigger_max_angle_deg_ = 0.0;
};

} // namespace rmcs_core::controller::shooting

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::shooting::DartLauncherController, rmcs_executor::Component)
