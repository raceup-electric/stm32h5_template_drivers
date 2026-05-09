#pragma once

#include <cstdint>

#include "common.hpp"
#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_tim.h"

namespace ru::driver {
class Timer;

struct opaque_timer {
 public:
  constexpr opaque_timer() noexcept = default;
  constexpr explicit opaque_timer(TIM_TypeDef* const p_instance) noexcept
      : m_p_instance(p_instance) {}

 private:
  friend class Timer;

  result init(TIM_HandleTypeDef* p_handle) const noexcept;
  result stop(TIM_HandleTypeDef* p_handle) const noexcept;
  expected::expected<uint64_t, result> time_now(
      const TIM_HandleTypeDef* p_handle) const noexcept;

  TIM_TypeDef* m_p_instance{nullptr};
};
}  // namespace ru::driver
