#include <cmath>
#include <cstdint>
#include <array>
#include <algorithm>
#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/dart_shooting_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/tick_timer.hpp>

namespace rmcs_core::controller::shooting {
class ShootingDartController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ShootingDartController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/switch/right", switch_right_);
        for (std::size_t index = 0; index < 4; ++index)
            register_output("/trigger/servo/servo_" + std::to_string(index + 1) + "/angle",
                trigger_servo_angle_[index], 90.0);
        register_output("/trigger/servo/fire_request_sequence", fire_request_sequence_, 0);
        register_input("/trigger/servo/response_sequence", servo_response_sequence_);
        register_input("/trigger/servo/response_command", servo_response_command_);
        register_input("/trigger/servo/response_status", servo_response_status_);
        register_input("/trigger/servo/response_id", servo_response_id_);
        register_output("/trigger/left_motor/release", left_motor_release_, false);
        register_output("/trigger/right_motor/release", right_motor_release_, false);
        register_input("/trigger/left_motor/angle", trigger_motor_angle_[0]);
        register_output("/trigger/left_motor/control_angle", trigger_motor_control_angle[0]);

        register_input("/trigger/right_motor/angle", trigger_motor_angle_[1]);
        register_output("/trigger/right_motor/control_angle", trigger_motor_control_angle[1]);
        register_input("/yaw_motor/angle", yaw_motor_angle_);
        register_input("/distance_motor/angle", distance_motor_angle_);
        register_output("/yaw_motor/control_angle", yaw_motor_control_angle_);
        register_output("/distance_motor/control_angle", distance_motor_control_angle_);
        register_output("/trigger/mode", mode_, rmcs_msgs::dart_shooting_mode::SHOOT);

        get_parameter("joystick_left_bias_y", joystick_left_bias_y_);
        get_parameter("joystick_sensitivity", joystick_sensitivity_);
        get_parameter("joystick_deadzone", joystick_deadzone_);
        get_parameter("switch_debounce_ticks", switch_debounce_ticks_);
        get_parameter("action_timeout_ticks", action_timeout_ticks_);
        get_parameter("servo_reload1_duration_ticks", reload_duration_ticks_[0]);
        get_parameter("servo_reload2_duration_ticks", reload_duration_ticks_[1]);
        get_parameter("servo_reload3_duration_ticks", reload_duration_ticks_[2]);
        get_parameter("servo_reload_first_duration_ticks", reload_first_duration_ticks_);
        get_parameter("servo_reload_third_duration_ticks", reload_third_duration_ticks_);
        get_parameter("servo_reload_fourth_duration_ticks", reload_fourth_duration_ticks_);
        load_servo_angles("servo_fire_angles", servo_fire_angle_);
        load_servo_angles("servo_safe_angles", servo_safe_angle_);
        load_servo_angles("servo_reload1_angles", servo_reload1_angle_);
        load_servo_angles("servo_reload2_angles", servo_reload2_angle_);
        load_servo_angles("servo_reload3_angles", servo_reload3_angle_);
        get_parameter("yaw_motor_velocity_limit", yaw_motor_velocity_limit_);
        get_parameter("distance_motor_velocity_limit", distance_motor_velocity_limit_);
    }

    void update() override {
        using namespace rmcs_msgs;
        auto switch_right = *switch_right_;
        auto switch_left = *switch_left_;
        if (!target_angle_initialized_) {
            trigger_motor_target_angle[0] = (*trigger_motor_angle_[0]);
            trigger_motor_target_angle[1] = (*trigger_motor_angle_[1]);

            *trigger_motor_control_angle[0] = (trigger_motor_target_angle[0]);
            *trigger_motor_control_angle[1] = (trigger_motor_target_angle[1]);

            target_angle_initialized_ = true;
            yaw_motor_target_angle_ = *yaw_motor_angle_;
            distance_motor_target_angle_ = *distance_motor_angle_;
            candidate_switch_left_ = switch_left;
            stable_switch_left_ = switch_left;
            switch_left_stable_count_ = switch_debounce_ticks_;
            candidate_switch_right_ = switch_right;
            stable_switch_right_ = switch_right;
            switch_stable_count_ = switch_debounce_ticks_;
            left_trigger_armed_ = switch_left != Switch::MIDDLE;
            right_trigger_armed_ = switch_right != Switch::MIDDLE;
        }

        const bool remote_unavailable =
            switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN;

        const bool controls_disabled = switch_left == Switch::DOWN && switch_right == Switch::DOWN;

        if (remote_unavailable || controls_disabled) {
            if (controls_disabled) {
                fault_latched_ = false;
                if (!safety_locked_)
                    RCLCPP_INFO(get_logger(), "Both switches DOWN: enter safe lock");
            } else if (safety_locked_) {
                RCLCPP_WARN(get_logger(), "Remote unavailable: keep safe lock");
            }

            enter_safe_state();
            return;
        }

        const double yaw_control_velocity =
            (*joystick_right_).x() * yaw_motor_velocity_limit_;
        const double distance_control_velocity =
            (*joystick_right_).y() * distance_motor_velocity_limit_;
        yaw_motor_target_angle_ += yaw_control_velocity * control_period_;
        distance_motor_target_angle_ += distance_control_velocity * control_period_;
        *yaw_motor_control_angle_ = yaw_motor_target_angle_;
        *distance_motor_control_angle_ = distance_motor_target_angle_;
        update_action_feedback();
        update_reload_motion();
        update_fire_hold(switch_left);

        if (safety_locked_ && !fault_latched_
            && switch_left != Switch::DOWN && switch_right != Switch::DOWN) {
            safety_locked_ = false;
            left_trigger_armed_ = true;
            right_trigger_armed_ = true;
            RCLCPP_INFO(get_logger(), "Safety lock released: both switches leave DOWN");
        }

        double left_y = (*joystick_left_).y() - joystick_left_bias_y_;
        if (std::abs(left_y) < joystick_deadzone_)
            left_y = 0.0;

        if ((*left_motor_release_ || *right_motor_release_) && left_y != 0.0) {
            trigger_motor_target_angle[0] = *trigger_motor_angle_[0];
            trigger_motor_target_angle[1] = *trigger_motor_angle_[1];
            *left_motor_release_ = false;
            *right_motor_release_ = false;
            RCLCPP_INFO(get_logger(), "Trigger motors state: position control");
        }

        const double control_angle = 2*left_y * joystick_sensitivity_;

        trigger_motor_target_angle[0] -= control_angle;
        trigger_motor_target_angle[1] += control_angle;
        // for (int i = 0; i < 2; ++i) {
        //     if (trigger_motor_target_angle[i] > M_PI) {
        //         trigger_motor_target_angle[i] = -2 * M_PI + trigger_motor_target_angle[i];
        //     } else if (trigger_motor_target_angle[i] < -M_PI) {
        //         trigger_motor_target_angle[i] = 2 * M_PI + trigger_motor_target_angle[i];
        //     }
        //     *trigger_motor_control_angle[i] =
        //         trigger_motor_target_angle[i] - (*trigger_motor_angle_[i] - M_PI);
        //     if (*trigger_motor_control_angle[i] > M_PI) {
        //         *trigger_motor_control_angle[i] = -2 * M_PI + *trigger_motor_control_angle[i];
        //     } else if (*trigger_motor_control_angle[i] < -M_PI) {
        //         *trigger_motor_control_angle[i] = 2 * M_PI + *trigger_motor_control_angle[i];
        //     }
        // }
        *trigger_motor_control_angle[0] = trigger_motor_target_angle[0] ;
        *trigger_motor_control_angle[1] = trigger_motor_target_angle[1] ;

        const bool left_switch_middle_entered = update_switch_edge(
            switch_left, candidate_switch_left_, stable_switch_left_,
            switch_left_stable_count_, left_trigger_armed_);
        const auto previous_right_switch = stable_switch_right_;
        const bool right_switch_middle_entered = update_switch_edge(
            switch_right, candidate_switch_right_, stable_switch_right_,
            switch_stable_count_, right_trigger_armed_);
        const bool right_switch_down_entered = previous_right_switch != Switch::DOWN
            && stable_switch_right_ == Switch::DOWN;

        if (!safety_locked_ && !fault_latched_ && !waiting_for_completion_) {
            if (left_switch_middle_entered && switch_right != Switch::DOWN) {
                if (!motors_ready_for_fire()) {
                    RCLCPP_WARN(
                        get_logger(),
                        "FIRE requested while trigger motors are not ready; "
                        "servo action will continue, left_error=%.4f right_error=%.4f",
                        std::abs(trigger_motor_target_angle[0] - *trigger_motor_angle_[0]),
                        std::abs(trigger_motor_target_angle[1] - *trigger_motor_angle_[1]));
                }
                RCLCPP_INFO(get_logger(), "Left switch returned MIDDLE: trigger FIRE");
                *left_motor_release_ = true;
                *right_motor_release_ = true;
                RCLCPP_INFO(get_logger(), "Trigger motors state: released for FIRE");
                request_action(ServoCommand::FIRE);
            } else if (right_switch_down_entered && switch_left != Switch::DOWN) {
                RCLCPP_INFO(get_logger(), "Right switch entered DOWN: rollback reload");
                rollback_reload();
            } else if (right_switch_middle_entered && switch_left != Switch::DOWN) {
                RCLCPP_INFO(get_logger(), "Right switch returned MIDDLE: request reload");
                request_reload();
            }
        }

    }

private:
    enum class ServoCommand : std::uint8_t {
        FIRE = 0x01,
        RELOAD = 0x02,
        STOP = 0x03,
        NONE = 0xff
    };

    void enter_safe_state() {
        *left_motor_release_ = false;
        *right_motor_release_ = false;
        *yaw_motor_control_angle_ = *yaw_motor_angle_;
        *distance_motor_control_angle_ = *distance_motor_angle_;
        *mode_ = rmcs_msgs::dart_shooting_mode::SHOOT;
        set_servo_angles(servo_safe_angle_, 0x0f);

        waiting_for_completion_ = false;
        reload_ticks_remaining_ = 0;
        fire_hold_active_ = false;
        const bool was_locked = safety_locked_;
        safety_locked_ = true;
        left_trigger_armed_ = false;
        right_trigger_armed_ = false;
        if (!was_locked) {
            RCLCPP_INFO(get_logger(), "Shooting controller safe state active");
            RCLCPP_INFO(get_logger(), "Trigger motors state: hold position");
        }
    }

    bool motors_ready_for_fire() const {
        const double left_error = std::abs(
            trigger_motor_target_angle[0] - *trigger_motor_angle_[0]);
        const double right_error = std::abs(
            trigger_motor_target_angle[1] - *trigger_motor_angle_[1]);
        return left_error <= fire_position_tolerance_
            && right_error <= fire_position_tolerance_;
    }

    bool update_switch_edge(
        rmcs_msgs::Switch current,
        rmcs_msgs::Switch& candidate,
        rmcs_msgs::Switch& stable,
        int& stable_count,
        bool& armed) const {
        if (current != candidate) {
            candidate = current;
            stable_count = 0;
        }

        if (stable_count < switch_debounce_ticks_)
            ++stable_count;

        if (stable_count < switch_debounce_ticks_ || stable == candidate)
            return false;

        const auto previous = stable;
        stable = candidate;

        if (stable != rmcs_msgs::Switch::MIDDLE)
            armed = true;

        return armed && previous != rmcs_msgs::Switch::MIDDLE
            && stable == rmcs_msgs::Switch::MIDDLE;
    }

    void request_action(ServoCommand action) {
        waiting_action_ = action;
        waiting_for_completion_ = true;
        action_timeout_remaining_ = action_timeout_ticks_;
        response_sequence_before_action_ = *servo_response_sequence_;
        fire_hold_active_ = false;
        *fire_request_sequence_ = static_cast<std::uint8_t>(*fire_request_sequence_ + 1);
        constexpr std::uint8_t fire_servo_mask = 0x01;
        set_servo_angles(servo_fire_angle_, fire_servo_mask);
        RCLCPP_INFO(
            get_logger(),
            "Servo FIRE command: mask=0x%02x servo_1_target=%.1f response_sequence_before=%u",
            fire_servo_mask,
            servo_fire_angle_[0],
            static_cast<unsigned>(response_sequence_before_action_));

        if (action == ServoCommand::FIRE)
            *mode_ = rmcs_msgs::dart_shooting_mode::STEP_DOWN;
        else if (action == ServoCommand::RELOAD)
            *mode_ = rmcs_msgs::dart_shooting_mode::RELOAD;
    }

    void update_action_feedback() {
        if (!waiting_for_completion_) return;
        if (action_timeout_remaining_ > 0) --action_timeout_remaining_;
        const auto response_id = *servo_response_id_;
        const bool response_ok = *servo_response_command_ == 0x81
            && response_id >= 1 && response_id <= 4 && *servo_response_status_ == 0;
        const bool is_new_response = *servo_response_sequence_ != response_sequence_before_action_;
        if (response_ok && is_new_response) {
            RCLCPP_INFO(
                get_logger(),
                "Servo action acknowledged: id=%u sequence=%u status=0x%02x",
                static_cast<unsigned>(*servo_response_id_),
                static_cast<unsigned>(*servo_response_sequence_),
                static_cast<unsigned>(*servo_response_status_));
            waiting_for_completion_ = false;
            if (waiting_action_ == ServoCommand::FIRE) {
                fire_hold_active_ = true;
                RCLCPP_INFO(
                    get_logger(),
                    "Servo FIRE acknowledged: hold servo_1 at %.1f while left switch is MIDDLE",
                    servo_fire_angle_[0]);
            }
            *mode_ = rmcs_msgs::dart_shooting_mode::SHOOT;
        } else if ((*servo_response_command_ == 0x81 && *servo_response_status_ != 0)
                   || action_timeout_remaining_ == 0) {
            RCLCPP_ERROR(
                get_logger(),
                "Servo action failed or timed out: id=%u command=0x%02x sequence=%u status=0x%02x",
                static_cast<unsigned>(*servo_response_id_),
                static_cast<unsigned>(*servo_response_command_),
                static_cast<unsigned>(*servo_response_sequence_),
                static_cast<unsigned>(*servo_response_status_));
            waiting_for_completion_ = false;
            fire_hold_active_ = false;
            safety_locked_ = true;
            fault_latched_ = true;
            set_servo_angles(servo_safe_angle_, 0x0f);
            *mode_ = rmcs_msgs::dart_shooting_mode::SHOOT;
        }
    }

    void update_fire_hold(rmcs_msgs::Switch left_switch) {
        if (!fire_hold_active_ || left_switch == rmcs_msgs::Switch::MIDDLE)
            return;
        fire_hold_active_ = false;
        set_servo_angles(servo_safe_angle_, 0x0f);
        RCLCPP_INFO(get_logger(), "Servo FIRE hold released: return to safe angle 0.0");
    }

    void request_reload() {
        const auto stage = reload_count_ % 3;
        fire_hold_active_ = false;
        reload_servo_mask_ = stage == 0 ? 0x06 : 0x0c;
        apply_reload_stage(stage);
        const auto duration = reload_count_ == 0 ? reload_first_duration_ticks_
            : reload_count_ == 2 ? reload_third_duration_ticks_
            : reload_count_ == 3 ? reload_fourth_duration_ticks_
                                 : reload_duration_ticks_[stage];
        reload_ticks_remaining_ = std::max(1, duration);
        ++reload_count_;
        RCLCPP_INFO(
            get_logger(),
            "Dart reload #%zu: reload servos only, mask=0x%02x; servo_1 is unchanged",
            stage + 1,
            reload_servo_mask_);
        waiting_action_ = ServoCommand::NONE;
        waiting_for_completion_ = false;
        fault_latched_ = false;
        *mode_ = rmcs_msgs::dart_shooting_mode::RELOAD;
        RCLCPP_INFO(
            get_logger(), "Dart reload started; selected servos will return to safe angles");
    }

    void update_reload_motion() {
        if (reload_ticks_remaining_ <= 0)
            return;
        if (--reload_ticks_remaining_ > 0)
            return;
        set_servo_angles(servo_safe_angle_, reload_servo_mask_);
        *mode_ = rmcs_msgs::dart_shooting_mode::SHOOT;
        RCLCPP_INFO(get_logger(), "Dart reload complete: servos returned to safe angles");
    }

    void rollback_reload() {
        if (reload_count_ == 0) {
            RCLCPP_WARN(get_logger(), "Dart reload rollback ignored: no previous stage");
            return;
        }
        --reload_count_;
        waiting_for_completion_ = false;
        reload_ticks_remaining_ = 0;
        apply_reload_stage(reload_count_ % 3);
        *mode_ = rmcs_msgs::dart_shooting_mode::SHOOT;
        RCLCPP_WARN(get_logger(), "Dart reload rolled back to stage %zu", reload_count_ % 3 + 1);
    }

    void apply_reload_stage(std::size_t stage) {
        if (stage == 0)
            set_servo_angles(servo_reload1_angle_, 0x06);
        else if (stage == 1)
            set_servo_angles(servo_reload2_angle_, 0x0c);
        else
            set_servo_angles(servo_reload3_angle_, 0x0c);
    }

    void set_servo_angles(const std::array<double, 4>& angles, std::uint8_t mask) {
        for (std::size_t index = 0; index < 4; ++index)
            if (mask & (1u << index))
                *trigger_servo_angle_[index] = angles[index];
    }

    void load_servo_angles(const char* name, std::array<double, 4>& target) {
        std::vector<double> values;
        if (get_parameter(name, values) && values.size() == target.size())
            std::copy(values.begin(), values.end(), target.begin());
    }

private:
    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<double> trigger_motor_angle_[2];
    OutputInterface<double> trigger_servo_angle_[4];
    OutputInterface<std::uint8_t> fire_request_sequence_;
    InputInterface<uint8_t> servo_response_sequence_;
    InputInterface<uint8_t> servo_response_command_;
    InputInterface<uint8_t> servo_response_status_;
    InputInterface<uint8_t> servo_response_id_;
    OutputInterface<double> trigger_motor_control_angle[2];
    OutputInterface<double> yaw_motor_control_velocity_;
    OutputInterface<double> distance_motor_control_velocity_;
    InputInterface<double> yaw_motor_angle_;
    InputInterface<double> distance_motor_angle_;
    OutputInterface<double> yaw_motor_control_angle_;
    OutputInterface<double> distance_motor_control_angle_;
    OutputInterface<bool> left_motor_release_;
    OutputInterface<bool> right_motor_release_;

    double trigger_motor_target_angle[2] = {0.0, 0.0};
    bool target_angle_initialized_ = false;
    rmcs_msgs::Switch candidate_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch stable_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch candidate_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch stable_switch_right_ = rmcs_msgs::Switch::UNKNOWN;

    int switch_left_stable_count_ = 0;
    int switch_stable_count_ = 0;
    int switch_debounce_ticks_ = 20;
    int action_timeout_ticks_ = 1500;
    std::array<int, 3> reload_duration_ticks_{500, 500, 500};
    int reload_first_duration_ticks_ = 2000;
    int reload_third_duration_ticks_ = 4000;
    int reload_fourth_duration_ticks_ = 4000;
    int reload_ticks_remaining_ = 0;
    std::uint8_t reload_servo_mask_ = 0;
    int action_timeout_remaining_ = 0;
    std::uint8_t response_sequence_before_action_ = 0;
    ServoCommand waiting_action_ = ServoCommand::NONE;
    double fire_position_tolerance_ = 0.05;
    std::array<double, 4> servo_fire_angle_{0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> servo_safe_angle_{0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> servo_reload1_angle_{90.0, 90.0, 90.0, 90.0};
    std::array<double, 4> servo_reload2_angle_{90.0, 90.0, 90.0, 90.0};
    std::array<double, 4> servo_reload3_angle_{90.0, 90.0, 90.0, 90.0};
    std::size_t reload_count_ = 0;

    bool waiting_for_completion_ = false;
    bool fire_hold_active_ = false;
    bool safety_locked_ = false;
    bool fault_latched_ = false;
    bool left_trigger_armed_ = true;
    bool right_trigger_armed_ = true;

    double joystick_sensitivity_ = 0.016;
    double joystick_deadzone_ = 0.5;

    OutputInterface<rmcs_msgs::dart_shooting_mode> mode_;
    double joystick_left_bias_y_ = 0.0;
    double yaw_motor_velocity_limit_ = 40.0;
    double distance_motor_velocity_limit_ = 40.0;
    double yaw_motor_target_angle_ = 0.0;
    double distance_motor_target_angle_ = 0.0;
    static constexpr double control_period_ = 0.001;
};

} // namespace rmcs_core::controller::shooting

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::shooting::ShootingDartController, rmcs_executor::Component)
