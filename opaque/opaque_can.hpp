#pragma once

#include <cstdint>

namespace ru::driver {
enum class can_controller_key : uint8_t { Invalid, Fdcan1, Fdcan2 };

struct opaque_can {
  constexpr opaque_can() noexcept = default;
  constexpr explicit opaque_can(const can_controller_key key) noexcept : m_key(key) {}

  can_controller_key m_key{can_controller_key::Invalid};
};
}  // namespace ru::driver
