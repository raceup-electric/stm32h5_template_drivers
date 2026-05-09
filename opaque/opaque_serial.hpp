#pragma once

#include <cstddef>
#include <cstdint>

#include "common.hpp"


#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_uart.h"


namespace ru::driver {

inline void enable_serial_clock(USART_TypeDef* const p_instance) noexcept {
  if (p_instance == USART1) {
    __HAL_RCC_USART1_CLK_ENABLE();
  } else if (p_instance == USART2) {
    __HAL_RCC_USART2_CLK_ENABLE();
  } else if (p_instance == USART3) {
    __HAL_RCC_USART3_CLK_ENABLE();
  } else if (p_instance == UART4) {
    __HAL_RCC_UART4_CLK_ENABLE();
  }
}





  class Serial;

struct opaque_serial {
 public:
  constexpr opaque_serial() noexcept = default;

 private:
  friend class Serial;

  result init() noexcept;
  result stop() noexcept;
  result write(const uint8_t* p_data, size_t len, Timestamp timeout_uS) noexcept;
  result read(uint8_t* p_data, size_t len, Timestamp timeout_uS) noexcept;

  bool m_initialized{false};
};
}  // namespace ru::driver
