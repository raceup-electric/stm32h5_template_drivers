#pragma once

#include <cstdint>

#include "common.hpp"
#include "stm32h5xx_hal.h"  // IWYU pragma: keep

namespace ru::driver::stm32h5xx::cfg {
struct adc_config;
}

namespace ru::driver {

class Adc;

struct opaque_adc {
 public:
  constexpr opaque_adc() noexcept = default;
  constexpr explicit opaque_adc(ADC_HandleTypeDef* const p_handle) noexcept
      : m_p_handle(p_handle) {}

 private:
  friend class Adc;

  static void start() noexcept;
  result init(const stm32h5xx::cfg::adc_config& config) const noexcept;
  result stop() const noexcept;
  result read(uint16_t& r_value) const noexcept;
  result try_read(bool& r_has_value, uint16_t& r_value) const noexcept;

  ADC_HandleTypeDef* m_p_handle{nullptr};
};
}  // namespace ru::driver
