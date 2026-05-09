#include "opaque_gpio.hpp"

#include <stm32h5xx_hal_gpio.h>

#include <cassert>

#include "stm_common.hpp"

using namespace ru::driver;

namespace ru::driver {

result opaque_gpio::init() const noexcept {
  if (m_p_port == nullptr || m_pin == 0U) {
    return result::UNRECOVERABLE_ERROR;
  }

  init_pin(m_p_port, m_pin, m_mode, m_pull, m_speed);

  if (is_output()) {
    return set_level(false);
  }

  return result::OK;
}

result opaque_gpio::stop() const noexcept {
  if (m_p_port == nullptr || m_pin == 0U) {
    return result::UNRECOVERABLE_ERROR;
  }

  HAL_GPIO_DeInit(m_p_port, m_pin);
  return result::OK;
}

bool opaque_gpio::is_active() const noexcept {
  assert(m_p_port);
  return (HAL_GPIO_ReadPin(m_p_port, m_pin) == GPIO_PIN_SET) == m_is_active_set;
}

bool opaque_gpio::is_high() const noexcept {
  assert(m_p_port);

  return HAL_GPIO_ReadPin(m_p_port, m_pin) == GPIO_PIN_SET;
}

result opaque_gpio::set_level(const bool active) const noexcept {
  assert(m_p_port);
  if (!is_output()) {
    return result::UNRECOVERABLE_ERROR;
  }

  HAL_GPIO_WritePin(m_p_port, m_pin,
                    active == m_is_active_set ? GPIO_PIN_SET : GPIO_PIN_RESET);
  return result::OK;
}

result opaque_gpio::toggle() const noexcept {
  assert(m_p_port);
  if (!is_output()) {
    return result::UNRECOVERABLE_ERROR;
  }

  HAL_GPIO_TogglePin(m_p_port, m_pin);
  return result::OK;
}
}  // namespace ru::driver
