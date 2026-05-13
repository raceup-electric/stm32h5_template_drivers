#pragma once

#include <cstdint>

#include "common.hpp"
#include "mapping_types.hpp"
#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_tim.h"

namespace ru::driver {
class Pwm;

struct opaque_pwm {
 public:
  constexpr opaque_pwm() noexcept = default;
  constexpr opaque_pwm(TIM_HandleTypeDef* const p_handle,
                       const uint32_t channel) noexcept
      : m_p_handle(p_handle), m_channel(channel) {}

 private:
  friend class Pwm;

  result init(const stm32h5xx::cfg::pwm_config& config,
              uint32_t frequency_hz) noexcept;
  result enable() noexcept;
  result disable() noexcept;
  result deinit() noexcept;
  bool is_enabled() const noexcept;
  result update_duty_cycle(uint16_t duty_cycle_permille) noexcept;
  expected::expected<uint16_t, result> get_duty_cycle() const noexcept;

  TIM_HandleTypeDef* m_p_handle{nullptr};
  uint32_t m_channel{0U};
};
}  // namespace ru::driver
