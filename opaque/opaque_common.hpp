#pragma once

#include "common.hpp"

namespace ru::driver {
class Common;

struct opaque_common {
  result start() const noexcept;
};
}  // namespace ru::driver
