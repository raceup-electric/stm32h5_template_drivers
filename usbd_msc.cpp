#include "usbd_msc.hpp"

using namespace ru::driver;

namespace ru::driver {
result UsbdMsc::start() noexcept {
  return result::OK;
}

result UsbdMsc::init() noexcept {
  return result::OK;
}

result UsbdMsc::stop() noexcept {
  return result::OK;
}
}  // namespace ru::driver