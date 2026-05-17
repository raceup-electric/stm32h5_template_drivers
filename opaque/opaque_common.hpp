#pragma once

#include <cstdint>

namespace ru::driver {
enum class result : uint8_t;

struct opaque_common {
  result start() const noexcept;
};
}  // namespace ru::driver
