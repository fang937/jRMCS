#pragma once

#include <array>
#include <cstdint>
#include <stdexcept>

#include <librmcs/spec/c_board/gpio.hpp>

namespace rmcs_core::hardware::device::detail {

enum class CBoardGpioUsage : std::uint8_t { kUnused, kPwm, kButton };

class CBoardGpioUsageRegistry {
public:
    static void claim(
        const librmcs::spec::c_board::GpioDescriptor& gpio, CBoardGpioUsage usage) {
        const auto index = gpio.channel_index;
        if (index >= usages_.size())
            throw std::out_of_range{"CBoard GPIO channel is out of range"};

        auto& current = usages_[index];
        if (current != CBoardGpioUsage::kUnused && current != usage)
            throw std::logic_error{"CBoard GPIO already enabled for another device function"};
        current = usage;
    }

    static void release(
        const librmcs::spec::c_board::GpioDescriptor& gpio, CBoardGpioUsage usage) noexcept {
        const auto index = gpio.channel_index;
        if (index < usages_.size() && usages_[index] == usage)
            usages_[index] = CBoardGpioUsage::kUnused;
    }

private:
    static inline std::array<CBoardGpioUsage, 7> usages_{};
};

} // namespace rmcs_core::hardware::device::detail
