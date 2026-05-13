#include "opaque_gpio.hpp"

#include <stm32h5xx_hal_gpio.h>

#include <cassert>

#include "stm_common.hpp"

using namespace ru::driver;

namespace ru::driver {
namespace {
bool is_output_mode(GPIO_TypeDef* const p_port, const uint16_t pin) noexcept {
  if (p_port == nullptr || pin == 0U) {
    return false;
  }

  const auto pin_index = static_cast<uint32_t>(__builtin_ctz(pin));
  const auto mode = (p_port->MODER >> (pin_index * 2U)) & 0x3U;
  return mode == 0x1U;
}

bool is_output_init(const GPIO_InitTypeDef& init) noexcept {
  return init.Mode == GPIO_MODE_OUTPUT_PP || init.Mode == GPIO_MODE_OUTPUT_OD;
}
}  // namespace

result opaque_gpio::init(const GPIO_InitTypeDef& init) const noexcept {
  if (m_p_port == nullptr || m_pin == 0U) {
    return result::UNRECOVERABLE_ERROR;
  }

  init_pin(m_p_port, init);

  if (is_output_init(init)) {
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
  return (HAL_GPIO_ReadPin(m_p_port, m_pin) == GPIO_PIN_SET) == m_active_high;
}

bool opaque_gpio::is_high() const noexcept {
  assert(m_p_port);
  return HAL_GPIO_ReadPin(m_p_port, m_pin) == GPIO_PIN_SET;
}

result opaque_gpio::set_level(const bool active) const noexcept {
  assert(m_p_port);
  if (!is_output_mode(m_p_port, m_pin)) {
    return result::UNRECOVERABLE_ERROR;
  }

  HAL_GPIO_WritePin(m_p_port, m_pin,
                    active == m_active_high ? GPIO_PIN_SET : GPIO_PIN_RESET);
  return result::OK;
}

result opaque_gpio::toggle() const noexcept {
  assert(m_p_port);
  if (!is_output_mode(m_p_port, m_pin)) {
    return result::UNRECOVERABLE_ERROR;
  }

  HAL_GPIO_TogglePin(m_p_port, m_pin);
  return result::OK;
}
}  // namespace ru::driver
