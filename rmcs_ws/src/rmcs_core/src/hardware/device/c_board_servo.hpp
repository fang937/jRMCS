#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <librmcs/board/c_board.hpp>
#include <librmcs/data/datas.hpp>
#include <librmcs/spec/c_board/gpio.hpp>

#include "hardware/device/c_board_gpio_usage.hpp"

namespace rmcs_core::hardware::device {

/** Servo adapter for librmcs CBoard PWM GPIOs. */
class CBoardServo {
public:
    struct Config {
        double min_angle = 0.0;
        double max_angle = 180.0;
        std::uint32_t min_pulse_us = 500;
        std::uint32_t max_pulse_us = 2500;

        // CBoard TIM1/TIM8: 168 MHz timer clock / 56 / 60000 = 50 Hz.
        static constexpr std::uint32_t kPwmPeriodUs = 20'000;
    };

    CBoardServo()
        : CBoardServo(Config{}) {}

    explicit CBoardServo(Config config)
        : config_(config) {
        config_.max_angle = std::max(config_.min_angle, config_.max_angle);
        config_.max_pulse_us = std::max(config_.min_pulse_us, config_.max_pulse_us);
        angle_ = config_.min_angle;
    }

    CBoardServo(const CBoardServo&) = delete;
    CBoardServo& operator=(const CBoardServo&) = delete;

    ~CBoardServo() { disable(); }

    void enable(const librmcs::spec::c_board::GpioDescriptor& gpio) {
        if (enabled_)
            throw std::logic_error{"CBoard servo is already enabled"};
        detail::CBoardGpioUsageRegistry::claim(gpio, detail::CBoardGpioUsage::kPwm);
        gpio_ = &gpio;
        enabled_ = true;
    }

    void disable() noexcept {
        if (!enabled_)
            return;
        detail::CBoardGpioUsageRegistry::release(*gpio_, detail::CBoardGpioUsage::kPwm);
        gpio_ = nullptr;
        enabled_ = false;
    }

    void set_angle(double angle) noexcept {
        angle_ = std::clamp(angle, config_.min_angle, config_.max_angle);
    }

    [[nodiscard]] double angle() const noexcept { return angle_; }

    [[nodiscard]] std::uint32_t pulse_width_us() const noexcept {
        const auto span = config_.max_angle - config_.min_angle;
        if (span <= 0.0)
            return config_.min_pulse_us;

        const auto ratio = (angle_ - config_.min_angle) / span;
        return static_cast<std::uint32_t>(std::lround(
            static_cast<double>(config_.min_pulse_us)
            + ratio * static_cast<double>(config_.max_pulse_us - config_.min_pulse_us)));
    }

    [[nodiscard]] std::uint16_t duty16() const noexcept {
        return static_cast<std::uint16_t>(
            (static_cast<std::uint64_t>(pulse_width_us()) * 65535U
             + Config::kPwmPeriodUs / 2U)
            / Config::kPwmPeriodUs);
    }

    void write(
        librmcs::board::CBoard::PacketBuilder& builder,
        const librmcs::spec::c_board::GpioDescriptor& gpio) const {
        builder.gpio_analog_write(gpio, {.value = duty16()});
    }

    void write(librmcs::board::CBoard::PacketBuilder& builder) const {
        require_enabled();
        write(builder, *gpio_);
    }

    void set_angle(
        librmcs::board::CBoard::PacketBuilder& builder,
        const librmcs::spec::c_board::GpioDescriptor& gpio, double angle) {
        set_angle(angle);
        write(builder, gpio);
    }

    void set_angle(librmcs::board::CBoard::PacketBuilder& builder, double angle) {
        set_angle(angle);
        write(builder);
    }

    [[nodiscard]] bool enabled() const noexcept { return enabled_; }
    [[nodiscard]] std::uint8_t channel_index() const {
        require_enabled();
        return gpio_->channel_index;
    }

private:
    void require_enabled() const {
        if (!enabled_)
            throw std::logic_error{"CBoard servo must be enabled before use"};
    }

    Config config_;
    double angle_ = 0.0;
    const librmcs::spec::c_board::GpioDescriptor* gpio_ = nullptr;
    bool enabled_ = false;
};

} // namespace rmcs_core::hardware::device
