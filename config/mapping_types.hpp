#pragma once

#include <stdint.h>
#include <variant>

#include "stm32h5xx_hal.h"

namespace ru::driver::stm32h5xx::cfg {
struct gpio_config {
  uintptr_t port_base;
  bool active_high;
  GPIO_InitTypeDef init;

  GPIO_TypeDef* port() const noexcept {
    return reinterpret_cast<GPIO_TypeDef*>(port_base);
  }

  uint16_t pin() const noexcept {
    return static_cast<uint16_t>(init.Pin);
  }

  bool is_output() const noexcept {
    return init.Mode == GPIO_MODE_OUTPUT_PP || init.Mode == GPIO_MODE_OUTPUT_OD;
  }
};

struct adc_config {
  struct polling_backend_config {
    ADC_InitTypeDef init;
    ADC_ChannelConfTypeDef channel_init;
  };

  struct dma_backend_config {
    ADC_InitTypeDef init;
    ADC_ChannelConfTypeDef channel_init;
    uint32_t request;
    uintptr_t dma_channel_base;
    IRQn_Type irq;

    DMA_Channel_TypeDef* dma_channel() const noexcept {
      return reinterpret_cast<DMA_Channel_TypeDef*>(dma_channel_base);
    }
  };

  using backend_config = std::variant<polling_backend_config, dma_backend_config>;

  uintptr_t instance_base;
  uintptr_t port_base;
  GPIO_InitTypeDef pin_init;
  backend_config backend;

  ADC_TypeDef* instance() const noexcept {
    return reinterpret_cast<ADC_TypeDef*>(instance_base);
  }

  GPIO_TypeDef* port() const noexcept {
    return reinterpret_cast<GPIO_TypeDef*>(port_base);
  }

  uint16_t pin() const noexcept {
    return static_cast<uint16_t>(pin_init.Pin);
  }

  const polling_backend_config* polling() const noexcept {
    return std::get_if<polling_backend_config>(&backend);
  }

  const dma_backend_config* dma() const noexcept {
    return std::get_if<dma_backend_config>(&backend);
  }

  bool uses_dma() const noexcept { return dma() != nullptr; }

  static constexpr adc_config polling_config(
      const uintptr_t instance_base, const uintptr_t port_base,
      const GPIO_InitTypeDef& pin_init,
      const polling_backend_config& polling) noexcept {
    return adc_config{
        .instance_base = instance_base,
        .port_base = port_base,
        .pin_init = pin_init,
        .backend = polling,
    };
  }

  static constexpr adc_config dma_config(
      const uintptr_t instance_base, const uintptr_t port_base,
      const GPIO_InitTypeDef& pin_init, const dma_backend_config& dma) noexcept {
    return adc_config{
        .instance_base = instance_base,
        .port_base = port_base,
        .pin_init = pin_init,
        .backend = dma,
    };
  }
};

struct pwm_config {
  uintptr_t instance_base;
  uintptr_t port_base;
  uint32_t channel;
  uint32_t counter_clock_hz;
  GPIO_InitTypeDef pin_init;
  TIM_Base_InitTypeDef init;
  TIM_OC_InitTypeDef channel_init;

  TIM_TypeDef* instance() const noexcept {
    return reinterpret_cast<TIM_TypeDef*>(instance_base);
  }

  GPIO_TypeDef* port() const noexcept {
    return reinterpret_cast<GPIO_TypeDef*>(port_base);
  }

  uint16_t pin() const noexcept {
    return static_cast<uint16_t>(pin_init.Pin);
  }
};

struct timer_config {
  uintptr_t instance_base;
  uint32_t counter_clock_hz;
  TIM_Base_InitTypeDef init;

  TIM_TypeDef* instance() const noexcept {
    return reinterpret_cast<TIM_TypeDef*>(instance_base);
  }
};

struct can_config {
  uintptr_t instance_base;
  uintptr_t port_rx_base;
  uintptr_t port_tx_base;
  GPIO_InitTypeDef rx_pin_init;
  GPIO_InitTypeDef tx_pin_init;
  FDCAN_InitTypeDef init;

  FDCAN_GlobalTypeDef* instance() const noexcept {
    return reinterpret_cast<FDCAN_GlobalTypeDef*>(instance_base);
  }

  GPIO_TypeDef* port_rx() const noexcept {
    return reinterpret_cast<GPIO_TypeDef*>(port_rx_base);
  }

  GPIO_TypeDef* port_tx() const noexcept {
    return reinterpret_cast<GPIO_TypeDef*>(port_tx_base);
  }
};

struct nv_memory_config {
  enum class backend_kind : uint8_t { None, EmulatedEeprom, FlashRegion };

  backend_kind backend;
  uint32_t arg0;
  uint32_t size;
};
}  // namespace ru::driver::stm32h5xx::cfg
