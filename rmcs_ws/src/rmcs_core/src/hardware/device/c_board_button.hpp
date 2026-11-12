#pragma once

#include <cstdint>

#include <librmcs/board/c_board.hpp>
#include <librmcs/data/datas.hpp>
#include <librmcs/spec/c_board/gpio.hpp>

#include "hardware/device/c_board_gpio_usage.hpp"

namespace rmcs_core::hardware::device {

/** Active-low button input for a librmcs CBoard GPIO. */
class CBoardButton {
public:
    struct Config {
        std::uint16_t sample_period_ms = 10;
    };

    CBoardButton() = default;
    explicit CBoardButton(Config config)
        : config_(config) {}

    CBoardButton(const CBoardButton&) = delete;
    CBoardButton& operator=(const CBoardButton&) = delete;

    ~CBoardButton() { disable(); }

    void enable(
        librmcs::board::CBoard::PacketBuilder& builder,
        const librmcs::spec::c_board::GpioDescriptor& gpio) {
        if (enabled_)
            throw std::logic_error{"CBoard button is already enabled"};
        detail::CBoardGpioUsageRegistry::claim(gpio, detail::CBoardGpioUsage::kButton);
        gpio_ = &gpio;
        enabled_ = true;
        try {
            configure(builder, gpio);
        } catch (...) {
            disable();
            throw;
        }
    }

    void disable() noexcept {
        if (!enabled_)
            return;
        detail::CBoardGpioUsageRegistry::release(*gpio_, detail::CBoardGpioUsage::kButton);
        gpio_ = nullptr;
        enabled_ = false;
        pressed_ = false;
    }

    void configure(
        librmcs::board::CBoard::PacketBuilder& builder,
        const librmcs::spec::c_board::GpioDescriptor& gpio) const {
        builder.gpio_digital_read(
            gpio,
            {
                .period_ms = config_.sample_period_ms,
                .asap = true,
                .rising_edge = false,
                .falling_edge = false,
                .capture_timestamp = false,
                .pull = librmcs::data::GpioPull::kUp,
            });
    }

    void handle_read_result(const librmcs::data::GpioDigitalDataView& data) noexcept {
        pressed_ = !data.high;
    }

    [[nodiscard]] bool handles(
        const librmcs::spec::c_board::GpioDescriptor& gpio) const noexcept {
        return enabled_ && gpio_->channel_index == gpio.channel_index;
    }

    [[nodiscard]] bool pressed() const noexcept { return pressed_; }
    [[nodiscard]] bool powered() const noexcept { return pressed_; }
    [[nodiscard]] bool enabled() const noexcept { return enabled_; }

private:
    Config config_{};
    bool pressed_ = false;
    const librmcs::spec::c_board::GpioDescriptor* gpio_ = nullptr;
    bool enabled_ = false;
};

} // namespace rmcs_core::hardware::device
