#include "serial.hpp"
#include "usb/usb_cdc.h"

using namespace ru::driver;

namespace ru::driver {
Serial::Serial(const SerialId id) noexcept : m_id(id), m_opaque() {
}

result Serial::start() noexcept {
  return ru_stm32_usb_cdc_start() == 0 ? result::OK : result::RECOVERABLE_ERROR;
}

result Serial::init() noexcept {
  return m_opaque.init();
}

result Serial::stop() noexcept {
  return m_opaque.stop();
}

result Serial::write(const uint8_t* const p_data, const size_t len,
                     const Timestamp timeout_uS) noexcept {
  if (p_data == nullptr && len != 0U) {
    return result::RECOVERABLE_ERROR;
  }

  return m_opaque.write(p_data, len, timeout_uS);
}

result Serial::read(uint8_t* const p_data, const size_t len,
                    const Timestamp timeout_uS) noexcept {
  if (p_data == nullptr && len != 0U) {
    return result::RECOVERABLE_ERROR;
  }

  if (p_data == nullptr) {
    return result::OK;
  }

  return m_opaque.read(p_data, len, timeout_uS);
}
}  // namespace ru::driver
