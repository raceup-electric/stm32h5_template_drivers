#pragma once

#include <common.hpp>
#include <cstdint>

#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_adc_ex.h"
#include "stm32h5xx_hal_gpio.h"
#include "stm32h5xx_hal_gpio_ex.h"
#include "stm32h5xx_hal_uart_ex.h"

namespace ru::driver {

// GENERAL STUFF

constexpr result from_hal_status(const HAL_StatusTypeDef status) noexcept {
  switch (status) {
    case HAL_OK:
      return result::OK;
    case HAL_ERROR:
      return result::UNRECOVERABLE_ERROR;
    case HAL_BUSY:
      return result::RECOVERABLE_ERROR;
    case HAL_TIMEOUT:
      return result::RECOVERABLE_ERROR;
    default:
      return result::UNRECOVERABLE_ERROR;
  }
}

constexpr uint32_t us_to_ms(const Timestamp uS) noexcept {
  return static_cast<uint32_t>((uS + 999) / 1000);
}

// GPIO BEACUSE YES

static inline void enable_gpio_clock(const GPIO_TypeDef* port) noexcept {
  switch (reinterpret_cast<uintptr_t>(port)) {
    case GPIOA_BASE:
      __HAL_RCC_GPIOA_CLK_ENABLE();
      break;
    case GPIOB_BASE:
      __HAL_RCC_GPIOB_CLK_ENABLE();
      break;
    case GPIOC_BASE:
      __HAL_RCC_GPIOC_CLK_ENABLE();
      break;
    case GPIOD_BASE:
      __HAL_RCC_GPIOD_CLK_ENABLE();
      break;
    case GPIOE_BASE:
      __HAL_RCC_GPIOE_CLK_ENABLE();
      break;
    case GPIOF_BASE:
      __HAL_RCC_GPIOF_CLK_ENABLE();
      break;
    case GPIOG_BASE:
      __HAL_RCC_GPIOG_CLK_ENABLE();
      break;
    case GPIOH_BASE:
      __HAL_RCC_GPIOH_CLK_ENABLE();
      break;
    case GPIOI_BASE:
      __HAL_RCC_GPIOI_CLK_ENABLE();
      break;
    default:
      break;
  }
}

inline void init_pin(GPIO_TypeDef* const p_port, const uint32_t pin,
                     const uint32_t mode = GPIO_MODE_INPUT,
                     const uint32_t pull = GPIO_NOPULL,
                     const uint32_t speed = GPIO_SPEED_FREQ_LOW,
                     const uint32_t alternate = 0) noexcept {
  enable_gpio_clock(p_port);

  GPIO_InitTypeDef init{};
  init.Pin = pin;
  init.Mode = mode;
  init.Pull = pull;
  init.Speed = speed;
  init.Alternate = alternate;
  HAL_GPIO_Init(p_port, &init);
}

// TIMER BEACUSE YES

static inline void enable_tim_clock(const TIM_TypeDef* instance) noexcept {
  switch (reinterpret_cast<uintptr_t>(instance)) {
    case TIM1_BASE:
      __HAL_RCC_TIM1_CLK_ENABLE();
      break;
    case TIM2_BASE:
      __HAL_RCC_TIM2_CLK_ENABLE();
      break;
    case TIM3_BASE:
      __HAL_RCC_TIM3_CLK_ENABLE();
      break;
    case TIM4_BASE:
      __HAL_RCC_TIM4_CLK_ENABLE();
      break;
    case TIM5_BASE:
      __HAL_RCC_TIM5_CLK_ENABLE();
      break;
    case TIM6_BASE:
      __HAL_RCC_TIM6_CLK_ENABLE();
      break;
    case TIM7_BASE:
      __HAL_RCC_TIM7_CLK_ENABLE();
      break;
    case TIM8_BASE:
      __HAL_RCC_TIM8_CLK_ENABLE();
      break;
    case TIM12_BASE:
      __HAL_RCC_TIM12_CLK_ENABLE();
      break;
    case TIM13_BASE:
      __HAL_RCC_TIM13_CLK_ENABLE();
      break;
    case TIM14_BASE:
      __HAL_RCC_TIM14_CLK_ENABLE();
      break;
    case TIM15_BASE:
      __HAL_RCC_TIM15_CLK_ENABLE();
      break;
    case TIM16_BASE:
      __HAL_RCC_TIM16_CLK_ENABLE();
      break;
    case TIM17_BASE:
      __HAL_RCC_TIM17_CLK_ENABLE();
      break;
    default:
      break;
  }
}

}  // namespace ru::driver
