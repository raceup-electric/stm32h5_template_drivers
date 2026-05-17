#pragma once

#include <cstddef>
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
    struct continuous_trigger_config {};

    struct timer_trigger_config {
      uintptr_t timer_instance_base;
      uint32_t counter_clock_hz;
      uint32_t frequency_hz;
      TIM_Base_InitTypeDef timer_init;
      TIM_MasterConfigTypeDef master_config;

      TIM_TypeDef* timer_instance() const noexcept {
        return reinterpret_cast<TIM_TypeDef*>(timer_instance_base);
      }
    };

    using trigger_config =
        std::variant<continuous_trigger_config, timer_trigger_config>;

    ADC_InitTypeDef init;
    ADC_ChannelConfTypeDef channel_init;
    std::size_t frame_count;
    uint32_t request;
    uintptr_t dma_channel_base;
    IRQn_Type irq;
    trigger_config trigger;

    DMA_Channel_TypeDef* dma_channel() const noexcept {
      return reinterpret_cast<DMA_Channel_TypeDef*>(dma_channel_base);
    }

    constexpr const continuous_trigger_config* continuous_trigger() const noexcept {
      return std::get_if<continuous_trigger_config>(&trigger);
    }

    constexpr const timer_trigger_config* timer_trigger() const noexcept {
      return std::get_if<timer_trigger_config>(&trigger);
    }

    static constexpr dma_backend_config continuous(
        const ADC_InitTypeDef& init, const ADC_ChannelConfTypeDef& channel_init,
        const std::size_t frame_count, const uint32_t request,
        const uintptr_t dma_channel_base, const IRQn_Type irq) noexcept {
      return dma_backend_config{
          .init = init,
          .channel_init = channel_init,
          .frame_count = frame_count,
          .request = request,
          .dma_channel_base = dma_channel_base,
          .irq = irq,
          .trigger = continuous_trigger_config{},
      };
    }

    static constexpr dma_backend_config timer_triggered(
        const ADC_InitTypeDef& init, const ADC_ChannelConfTypeDef& channel_init,
        const std::size_t frame_count, const uint32_t request,
        const uintptr_t dma_channel_base, const IRQn_Type irq,
        const timer_trigger_config& trigger) noexcept {
      return dma_backend_config{
          .init = init,
          .channel_init = channel_init,
          .frame_count = frame_count,
          .request = request,
          .dma_channel_base = dma_channel_base,
          .irq = irq,
          .trigger = trigger,
      };
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

  constexpr const polling_backend_config* polling() const noexcept {
    return std::get_if<polling_backend_config>(&backend);
  }

  constexpr const dma_backend_config* dma() const noexcept {
    return std::get_if<dma_backend_config>(&backend);
  }

  constexpr bool uses_dma() const noexcept { return dma() != nullptr; }

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
  uintptr_t port_base;
  uint16_t pin;
  uint32_t pull;
  uint32_t alternate;
  struct general_purpose_backend_config {
    uintptr_t instance_base;
    uint32_t channel;
    TIM_Base_InitTypeDef init;
    TIM_OC_InitTypeDef channel_init;

    TIM_TypeDef* instance() const noexcept {
      return reinterpret_cast<TIM_TypeDef*>(instance_base);
    }
  };

  struct low_power_backend_config {
    uintptr_t instance_base;
    uint32_t source_clock_hz;
    uint32_t channel;
    LPTIM_InitTypeDef init;
    LPTIM_OC_ConfigTypeDef channel_init;

    LPTIM_TypeDef* instance() const noexcept {
      return reinterpret_cast<LPTIM_TypeDef*>(instance_base);
    }
  };

  using backend_config =
      std::variant<general_purpose_backend_config, low_power_backend_config>;

  backend_config backend;

  GPIO_TypeDef* port() const noexcept {
    return reinterpret_cast<GPIO_TypeDef*>(port_base);
  }

  const general_purpose_backend_config* general_purpose() const noexcept {
    return std::get_if<general_purpose_backend_config>(&backend);
  }

  const low_power_backend_config* low_power() const noexcept {
    return std::get_if<low_power_backend_config>(&backend);
  }

  static constexpr pwm_config general_purpose_config(
      const uintptr_t port_base, const uint16_t pin, const uint32_t pull,
      const uint32_t alternate,
      const general_purpose_backend_config& general_purpose) noexcept {
    return pwm_config{
        .port_base = port_base,
        .pin = pin,
        .pull = pull,
        .alternate = alternate,
        .backend = general_purpose,
    };
  }

  static constexpr pwm_config low_power_config(
      const uintptr_t port_base, const uint16_t pin, const uint32_t pull,
      const uint32_t alternate,
      const low_power_backend_config& low_power) noexcept {
    return pwm_config{
        .port_base = port_base,
        .pin = pin,
        .pull = pull,
        .alternate = alternate,
        .backend = low_power,
    };
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
