#include "opaque_serial.hpp"

#include "usb/usb_cdc.h"

using namespace ru::driver;

namespace ru::driver {
result opaque_serial::init() noexcept {
  if (m_initialized) {
    return result::OK;
  }

  const auto status =
      ru_stm32_usb_cdc_init() == 0 ? result::OK : result::RECOVERABLE_ERROR;
  if (status == result::OK) {
    m_initialized = true;
  }

  return status;
}

result opaque_serial::stop() noexcept {
  if (!m_initialized) {
    return result::OK;
  }

  const auto status =
      ru_stm32_usb_cdc_stop() == 0 ? result::OK : result::RECOVERABLE_ERROR;
  if (status == result::OK) {
    m_initialized = false;
  }

  return status;
}

result opaque_serial::write(const uint8_t* const p_data, const size_t len,
                            const Timestamp timeout_uS) noexcept {
  if (!m_initialized) {
    return result::RECOVERABLE_ERROR;
  }

  return ru_stm32_usb_cdc_write(p_data, len, timeout_uS) == 0
             ? result::OK
             : result::RECOVERABLE_ERROR;
}

result opaque_serial::read(uint8_t* const p_data, const size_t len,
                           const Timestamp timeout_uS) noexcept {
  (void)p_data;
  (void)timeout_uS;

  if (!m_initialized) {
    return result::RECOVERABLE_ERROR;
  }

  return len == 0U ? result::OK : result::RECOVERABLE_ERROR;
}
}  // namespace ru::driver
