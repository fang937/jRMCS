#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

#include <librmcs/data/datas.hpp>

namespace rmcs_core::hardware::device {

/**
 * A hobby servo driven by a cboard PWM GPIO.
 *
 * The GPIO protocol carries a 16-bit duty-cycle value, not a pulse width.
 * Supply the PWM period configured by the target board when constructing this
 * device. The default pulse endpoints follow flying-bro's servo mapping.
 */
class Servo {
public:
    struct Config {
        static constexpr std::uint32_t kRmcsBoardPwmPeriodUs = 20'000;

        double min_angle = 0.0;
        double max_angle = 180.0;
        std::uint32_t min_pulse_us = 500;
        std::uint32_t max_pulse_us = 2500;
        std::uint32_t pwm_period_us = kRmcsBoardPwmPeriodUs;
    };

    Servo()
        : Servo(Config{}) {}

    explicit Servo(Config config)
        : config_(config) {
        config_.max_angle = std::max(config_.min_angle, config_.max_angle);
        config_.max_pulse_us = std::max(config_.min_pulse_us, config_.max_pulse_us);
        if (config_.pwm_period_us == 0)
            throw std::invalid_argument{"Servo PWM period must be non-zero"};
        set_angle(config_.min_angle);
    }

    void set_angle(double angle) noexcept {
        angle_ = std::clamp(angle, config_.min_angle, config_.max_angle);
    }

    [[nodiscard]] double angle() const noexcept { return angle_; }

    [[nodiscard]] std::uint32_t pulse_width_us() const noexcept {
        const auto angle_span = config_.max_angle - config_.min_angle;
        if (angle_span <= 0.0)
            return config_.min_pulse_us;

        const auto ratio = (angle_ - config_.min_angle) / angle_span;
        const auto pulse = static_cast<double>(config_.min_pulse_us)
            + ratio * static_cast<double>(config_.max_pulse_us - config_.min_pulse_us);
        return static_cast<std::uint32_t>(std::lround(pulse));
    }

    /** Return the cboard's 16-bit duty-cycle value for gpio_analog_write. */
    [[nodiscard]] std::uint16_t duty16() const noexcept {
        const auto duty = static_cast<std::uint64_t>(pulse_width_us()) * 65535U;
        return static_cast<std::uint16_t>(
            std::min<std::uint64_t>(65535U, (duty + config_.pwm_period_us / 2U)
                                                   / config_.pwm_period_us));
    }

    template <typename PacketBuilder, typename GpioDescriptor>
    void write(PacketBuilder& builder, const GpioDescriptor& gpio) const {
        builder.gpio_analog_write(gpio, {.value = duty16()});
    }

    template <typename PacketBuilder, typename GpioDescriptor>
    void set_angle(
        PacketBuilder& builder, const GpioDescriptor& gpio, double angle) {
        set_angle(angle);
        write(builder, gpio);
    }

private:
    Config config_;
    double angle_ = 0.0;
};

} // namespace rmcs_core::hardware::device
