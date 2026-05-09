#pragma once

#include <cstdint>

#include "common.hpp"
#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_adc.h"

namespace ru::driver {

inline void enable_adc_clock(ADC_TypeDef* const p_instance) noexcept {
  (void)p_instance;
  __HAL_RCC_ADC_CLK_ENABLE();
}






class Adc;

struct opaque_adc {
 public:
  constexpr opaque_adc() noexcept = default;
  constexpr opaque_adc(ADC_TypeDef* const p_instance, GPIO_TypeDef* const p_port,
                       const uint16_t pin, const uint32_t channel) noexcept
      : m_p_instance(p_instance),
        m_p_port(p_port),
        m_pin(pin),
        m_channel(channel) {}

 private:
  friend class Adc;

  result init(ADC_HandleTypeDef* p_handle) const noexcept;
  result stop(ADC_HandleTypeDef* p_handle) const noexcept;
  result read(ADC_HandleTypeDef* p_handle, uint16_t& r_value) const noexcept;
  result try_read(ADC_HandleTypeDef* p_handle, bool& r_has_value,
                  uint16_t& r_value) const noexcept;

  ADC_TypeDef* m_p_instance{nullptr};
  GPIO_TypeDef* m_p_port{nullptr};
  uint16_t m_pin{0U};
  uint32_t m_channel{0U};
};
}  // namespace ru::driver
