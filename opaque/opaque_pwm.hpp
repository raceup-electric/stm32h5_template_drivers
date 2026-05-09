#pragma once

#include <cstdint>

#include "common.hpp"
#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_tim.h"

namespace ru::driver {
class Pwm;

struct opaque_pwm {
 public:
  constexpr opaque_pwm() noexcept = default;
  constexpr opaque_pwm(TIM_TypeDef* const p_instance, GPIO_TypeDef* const p_port,
                       const uint16_t pin, const uint32_t channel,
                       const uint32_t alternate) noexcept
      : m_p_instance(p_instance),
        m_p_port(p_port),
        m_pin(pin),
        m_channel(channel),
        m_alternate(alternate) {}

 private:
  friend class Pwm;

  result init(TIM_HandleTypeDef* p_handle, uint32_t frequency_hz) noexcept;
  result start(TIM_HandleTypeDef* p_handle) noexcept;
  result stop(TIM_HandleTypeDef* p_handle) noexcept;
  result deinit(TIM_HandleTypeDef* p_handle) noexcept;
  bool is_enabled(const TIM_HandleTypeDef* p_handle) const noexcept;
  result update_duty_cycle(TIM_HandleTypeDef* p_handle, uint16_t duty_cycle_permille) noexcept;
  expected::expected<uint16_t, result> get_duty_cycle(const TIM_HandleTypeDef* p_handle) const noexcept;

  TIM_TypeDef* m_p_instance{nullptr};
  GPIO_TypeDef* m_p_port{nullptr};
  uint16_t m_pin{0U};
  uint32_t m_channel{0U};
  uint32_t m_alternate{0U};
  uint32_t m_frequency_hz{1000U};
};
}  // namespace ru::driver
