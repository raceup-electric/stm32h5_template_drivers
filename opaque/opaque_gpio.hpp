#pragma once

#include <cstdint>

#include "common.hpp"
#include "stm32h5xx_hal.h"

namespace ru::driver {
class Gpio;

struct opaque_gpio {
 public:
  constexpr opaque_gpio() noexcept = default;

  constexpr opaque_gpio(GPIO_TypeDef* const p_port, const uint16_t pin,
                        const bool is_active_high, const uint32_t mode,
                        const uint32_t pull,
                        const uint32_t speed = GPIO_SPEED_FREQ_LOW) noexcept
      : m_p_port(p_port),
        m_pin(pin),
        m_is_active_set(is_active_high),
        m_mode(mode),
        m_pull(pull),
        m_speed(speed) {}

  result init() const noexcept;
  result stop() const noexcept;
  bool is_active() const noexcept;
  bool is_high() const noexcept;
  result set_level(bool active) const noexcept;
  result toggle() const noexcept;
  constexpr bool active_high() const noexcept { return m_is_active_set; }
  constexpr bool is_output() const noexcept {
    return m_mode == GPIO_MODE_OUTPUT_PP || m_mode == GPIO_MODE_OUTPUT_OD;
  }

  GPIO_TypeDef* m_p_port{nullptr};
  uint16_t m_pin{0U};
  bool m_is_active_set : 1 {true};
  uint32_t m_mode{GPIO_MODE_INPUT};
  uint32_t m_pull{GPIO_NOPULL};
  uint32_t m_speed{GPIO_SPEED_FREQ_LOW};
};
}  // namespace ru::driver
