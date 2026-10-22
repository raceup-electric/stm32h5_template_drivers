#pragma once

#include <cstddef>
#include <stdint.h>
#include <variant>

#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_adc_ex.h"

namespace ru::driver::stm32h5xx::cfg {
enum class adc_dma_backend {
  average_since_read,
  fixed_window_average,
};

enum class adc_acquisition_mode {
  polling,
  free_running,
  timer_triggered,
};

inline constexpr uint32_t kDefaultAdcPollTimeoutMs = 2U;
inline constexpr uint16_t kDefaultAdcSampleMax = 4095U;

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

struct adc_pipeline_config {
  uintptr_t instance_base;
  ADC_InitTypeDef init;
  const ADC_ChannelConfTypeDef* channel_sequence;
  std::size_t channel_sequence_length;
  adc_acquisition_mode acquisition_mode;
  std::size_t dma_frame_count;
  adc_dma_backend dma_backend;
  std::size_t dma_window_width;
  std::size_t dma_window_hop;
  uint32_t dma_request;
  uintptr_t dma_channel_base;
  IRQn_Type dma_irq;
  uintptr_t trigger_timer_instance_base;
  uint32_t trigger_counter_clock_hz;
  uint32_t trigger_frequency_hz;
  uint32_t adc_trigger;
  uint32_t adc_trigger_edge;
  TIM_Base_InitTypeDef trigger_timer_init;
  TIM_MasterConfigTypeDef trigger_master_config;

  ADC_TypeDef* instance() const noexcept {
    return reinterpret_cast<ADC_TypeDef*>(instance_base);
  }

  DMA_Channel_TypeDef* dma_channel() const noexcept {
    return reinterpret_cast<DMA_Channel_TypeDef*>(dma_channel_base);
  }

  TIM_TypeDef* trigger_timer_instance() const noexcept {
    return reinterpret_cast<TIM_TypeDef*>(trigger_timer_instance_base);
  }

  bool uses_dma() const noexcept {
    return dma_frame_count != 0U;
  }

  bool uses_timer_trigger() const noexcept {
    return acquisition_mode == adc_acquisition_mode::timer_triggered;
  }

  static constexpr adc_pipeline_config polling(
      const uintptr_t instance_base, const ADC_InitTypeDef& init,
      const ADC_ChannelConfTypeDef* channel_sequence,
      const std::size_t channel_sequence_length) noexcept {
    return adc_pipeline_config{
        .instance_base = instance_base,
        .init = init,
        .channel_sequence = channel_sequence,
        .channel_sequence_length = channel_sequence_length,
        .acquisition_mode = adc_acquisition_mode::polling,
        .dma_frame_count = 0U,
        .dma_backend = adc_dma_backend::average_since_read,
        .dma_window_width = 0U,
        .dma_window_hop = 0U,
        .dma_request = 0U,
        .dma_channel_base = 0U,
        .dma_irq = static_cast<IRQn_Type>(0),
        .trigger_timer_instance_base = 0U,
        .trigger_counter_clock_hz = 0U,
        .trigger_frequency_hz = 0U,
        .adc_trigger = ADC_SOFTWARE_START,
        .adc_trigger_edge = ADC_EXTERNALTRIGCONVEDGE_NONE,
        .trigger_timer_init = {},
        .trigger_master_config = {},
    };
  }

  static constexpr adc_pipeline_config dma(
      const uintptr_t instance_base, const ADC_InitTypeDef& init,
      const ADC_ChannelConfTypeDef* channel_sequence,
      const std::size_t channel_sequence_length,
      const std::size_t dma_frame_count, const uint32_t dma_request,
      const uintptr_t dma_channel_base, const IRQn_Type dma_irq,
      const adc_dma_backend dma_backend = adc_dma_backend::average_since_read,
      const std::size_t dma_window_width = 0U,
      const std::size_t dma_window_hop = 0U,
      const adc_acquisition_mode acquisition_mode =
          adc_acquisition_mode::free_running,
      const uintptr_t trigger_timer_instance_base = 0U,
      const uint32_t trigger_counter_clock_hz = 0U,
      const uint32_t trigger_frequency_hz = 0U,
      const uint32_t adc_trigger = ADC_SOFTWARE_START,
      const uint32_t adc_trigger_edge = ADC_EXTERNALTRIGCONVEDGE_NONE,
      const TIM_Base_InitTypeDef& trigger_timer_init = {},
      const TIM_MasterConfigTypeDef& trigger_master_config = {}) noexcept {
    return adc_pipeline_config{
        .instance_base = instance_base,
        .init = init,
        .channel_sequence = channel_sequence,
        .channel_sequence_length = channel_sequence_length,
        .acquisition_mode = acquisition_mode,
        .dma_frame_count = dma_frame_count,
        .dma_backend = dma_backend,
        .dma_window_width = dma_window_width,
        .dma_window_hop = dma_window_hop,
        .dma_request = dma_request,
        .dma_channel_base = dma_channel_base,
        .dma_irq = dma_irq,
        .trigger_timer_instance_base = trigger_timer_instance_base,
        .trigger_counter_clock_hz = trigger_counter_clock_hz,
        .trigger_frequency_hz = trigger_frequency_hz,
        .adc_trigger = adc_trigger,
        .adc_trigger_edge = adc_trigger_edge,
        .trigger_timer_init = trigger_timer_init,
        .trigger_master_config = trigger_master_config,
    };
  }
};

struct adc_config {
  const adc_pipeline_config* pipeline;
  uintptr_t port_base;
  GPIO_InitTypeDef pin_init;
  ADC_ChannelConfTypeDef channel_init;
  ADC_InjectionConfTypeDef injected_channel_init;
  uint32_t polling_timeout_ms;
  uint16_t sample_max;
  std::size_t sequence_index;
  bool uses_injected;

  GPIO_TypeDef* port() const noexcept {
    return reinterpret_cast<GPIO_TypeDef*>(port_base);
  }

  static constexpr adc_config polling_config(
      const adc_pipeline_config& pipeline, const uintptr_t port_base,
      const GPIO_InitTypeDef& pin_init,
      const ADC_ChannelConfTypeDef& channel_init,
      const uint32_t polling_timeout_ms = kDefaultAdcPollTimeoutMs) noexcept {
    return adc_config{
        .pipeline = &pipeline,
        .port_base = port_base,
        .pin_init = pin_init,
        .channel_init = channel_init,
        .injected_channel_init = {},
        .polling_timeout_ms = polling_timeout_ms,
        .sample_max = kDefaultAdcSampleMax,
        .sequence_index = 0U,
        .uses_injected = false,
    };
  }

  static constexpr adc_config dma_config(
      const adc_pipeline_config& pipeline, const uintptr_t port_base,
      const GPIO_InitTypeDef& pin_init,
      const ADC_ChannelConfTypeDef& channel_init,
      const std::size_t sequence_index) noexcept {
    return adc_config{
        .pipeline = &pipeline,
        .port_base = port_base,
        .pin_init = pin_init,
        .channel_init = channel_init,
        .injected_channel_init = {},
        .polling_timeout_ms = 0U,
        .sample_max = kDefaultAdcSampleMax,
        .sequence_index = sequence_index,
        .uses_injected = false,
    };
  }

  static constexpr adc_config injected_config(
      const adc_pipeline_config& pipeline, const uintptr_t port_base,
      const GPIO_InitTypeDef& pin_init,
      const ADC_InjectionConfTypeDef& injected_channel_init,
      const uint16_t sample_max,
      const uint32_t polling_timeout_ms = kDefaultAdcPollTimeoutMs) noexcept {
    return adc_config{
        .pipeline = &pipeline,
        .port_base = port_base,
        .pin_init = pin_init,
        .channel_init = {},
        .injected_channel_init = injected_channel_init,
        .polling_timeout_ms = polling_timeout_ms,
        .sample_max = sample_max,
        .sequence_index = 0U,
        .uses_injected = true,
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

struct usb_config {
  uint32_t task_priority;
  uint32_t task_period;
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
