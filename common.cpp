#include "common.hpp"

using namespace ru::driver;

namespace ru::driver {
Common::Common() noexcept = default;

result Common::start() noexcept {
  return opaque_common{}.start();
}
}  // namespace ru::driver
