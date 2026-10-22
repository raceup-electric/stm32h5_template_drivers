#include <algorithm>
#include <array>
#include <optional>

#include "adc.hpp"

#include "mapping.hpp"
#include "stm_common.hpp"
#include "stm32h5xx_hal_adc_ex.h"
#include "stm32h5xx_hal_dma_ex.h"
#include "stm32h5xx_hal_tim.h"
#include "stm32h5xx_hal_tim_ex.h"

using namespace ru::driver;

namespace ru::driver {
namespace {

using pipeline_config = stm32h5xx::cfg::adc_pipeline_config;
using channel_config = stm32h5xx::cfg::adc_config;

constexpr const channel_config* config_for(const AdcId id) noexcept {
  switch (id) {
#define RU_STM32H5XX_ADC_CONFIG(name, config) \
    case AdcId::name:                         \
      return &config;
    RU_STM32H5XX_ADC_MAP(RU_STM32H5XX_ADC_CONFIG)
#undef RU_STM32H5XX_ADC_CONFIG
    default:
      return nullptr;
  }
}

constexpr std::size_t dma_transfer_length(const pipeline_config& config) noexcept {
  return config.dma_frame_count == 0U ? 1U : config.dma_frame_count;
}

constexpr std::size_t dma_buffer_capacity_for_instance(
    const uintptr_t instance_base) noexcept {
  std::size_t capacity{1U};
#define RU_STM32H5XX_ADC_BUFFER_CAPACITY(name, config)                   \
  if (config.pipeline != nullptr &&                                      \
      config.pipeline->instance_base == instance_base) {                 \
    capacity = std::max(capacity, dma_transfer_length(*config.pipeline)); \
  }
  RU_STM32H5XX_ADC_MAP(RU_STM32H5XX_ADC_BUFFER_CAPACITY)
#undef RU_STM32H5XX_ADC_BUFFER_CAPACITY
  return capacity;
}

template <uintptr_t InstanceBase>
uint16_t* dma_buffer_data() noexcept {
  static uint16_t buffer[dma_buffer_capacity_for_instance(InstanceBase)]{};
  return buffer;
}

uint16_t* dma_buffer_for(const pipeline_config* const config) noexcept {
  if (config == nullptr) {
    return nullptr;
  }
  switch (config->instance_base) {
    case ADC1_BASE:
      return dma_buffer_data<ADC1_BASE>();
    case ADC2_BASE:
      return dma_buffer_data<ADC2_BASE>();
    default:
      return nullptr;
  }
}

std::size_t dma_buffer_capacity_for(const pipeline_config* const config) noexcept {
  return config == nullptr ? 0U
                           : dma_buffer_capacity_for_instance(config->instance_base);
}

opaque_adc make_opaque(const channel_config* const config) noexcept {
  return opaque_adc{config};
}

opaque_adc make_opaque(const AdcId id) noexcept {
  return make_opaque(config_for(id));
}

adc_pipeline_runtime* runtime_for_instance(ADC_TypeDef* const instance) noexcept {
  static adc_pipeline_runtime adc1_runtime{};
  static adc_pipeline_runtime adc2_runtime{};
  static adc_pipeline_runtime invalid_runtime{};

  switch (reinterpret_cast<uintptr_t>(instance)) {
    case ADC1_BASE:
      return &adc1_runtime;
    case ADC2_BASE:
      return &adc2_runtime;
    default:
      return &invalid_runtime;
  }
}

adc_pipeline_runtime* runtime_for_config(const channel_config* const config) noexcept {
  return config != nullptr && config->pipeline != nullptr
             ? runtime_for_instance(config->pipeline->instance())
             : nullptr;
}

adc_pipeline_runtime* runtime_from_handle(ADC_HandleTypeDef* const handle) noexcept {
  return handle != nullptr ? runtime_for_instance(handle->Instance) : nullptr;
}

std::size_t sequence_length(const pipeline_config& config) noexcept {
  return config.channel_sequence_length == 0U ? 1U : config.channel_sequence_length;
}

std::size_t window_width(const pipeline_config& config) noexcept {
  return config.dma_window_width == 0U ? dma_transfer_length(config)
                                       : config.dma_window_width;
}

std::size_t window_hop(const pipeline_config& config) noexcept {
  return config.dma_window_hop == 0U ? dma_transfer_length(config) / 2U
                                     : config.dma_window_hop;
}

bool uses_fixed_window_average(const adc_pipeline_runtime& runtime) noexcept {
  return runtime.p_config != nullptr &&
         runtime.p_config->dma_backend ==
             stm32h5xx::cfg::adc_dma_backend::fixed_window_average;
}

uint32_t lock_irq() noexcept {
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

void unlock_irq(const uint32_t primask) noexcept {
  __set_PRIMASK(primask);
}

void set_status(adc_pipeline_runtime& runtime, const adc_status status) noexcept {
  runtime.last_status = status;
}

uint32_t adc_instance_number(const uintptr_t instance_base) noexcept {
  switch (instance_base) {
    case ADC1_BASE:
      return 1U;
    case ADC2_BASE:
      return 2U;
    default:
      return 0U;
  }
}

uint32_t trigger_timer_number(const uintptr_t instance_base) noexcept {
  switch (instance_base) {
    case TIM6_BASE:
      return 6U;
    case TIM15_BASE:
      return 15U;
    default:
      return 0U;
  }
}

void initialize_diagnostic(adc_pipeline_runtime& runtime) noexcept {
  runtime.init_diagnostic = {};
  if (runtime.p_config == nullptr) {
    return;
  }

  runtime.init_diagnostic.adc_instance =
      adc_instance_number(runtime.p_config->instance_base);
  runtime.init_diagnostic.trigger_timer =
      trigger_timer_number(runtime.p_config->trigger_timer_instance_base);
  runtime.init_diagnostic.requested_frequency_hz =
      runtime.p_config->trigger_frequency_hz;
  runtime.init_diagnostic.dma_transfer_length = static_cast<uint32_t>(
      dma_transfer_length(*runtime.p_config));
  runtime.init_diagnostic.sequence_length = static_cast<uint32_t>(
      runtime.p_config->channel_sequence_length);
}

void set_init_diagnostic(
    adc_pipeline_runtime& runtime,
    const adc_init_stage stage,
    const adc_status status = adc_status::NOT_INITIALIZED,
    const uint32_t backend_code = 0U) noexcept {
  runtime.init_diagnostic.stage = stage;
  runtime.init_diagnostic.status = status;
  runtime.init_diagnostic.backend_code = backend_code;
  runtime.init_diagnostic.actual_frequency_hz = runtime.actual_frequency_hz;
}

bool valid_pipeline(const pipeline_config& config) noexcept {
  const auto transfer = dma_transfer_length(config);
  const auto channels = sequence_length(config);
  const auto width = window_width(config);
  const auto hop = window_hop(config);

  if (channels == 0U || channels > kAdcDmaMaxSequenceLength ||
      config.channel_sequence == nullptr || config.channel_sequence_length == 0U) {
    return false;
  }

  if (!config.uses_dma()) {
    return config.acquisition_mode == stm32h5xx::cfg::adc_acquisition_mode::polling;
  }

  if (transfer == 0U || (transfer % channels) != 0U ||
      (transfer % 2U) != 0U || width == 0U || width > transfer ||
      (width % channels) != 0U || hop != transfer / 2U ||
      (hop % channels) != 0U || config.dma_channel_base == 0U) {
    return false;
  }

  if (config.acquisition_mode ==
      stm32h5xx::cfg::adc_acquisition_mode::timer_triggered) {
    return config.trigger_timer_instance_base != 0U &&
           config.trigger_counter_clock_hz != 0U &&
           config.trigger_frequency_hz != 0U &&
           config.adc_trigger != ADC_SOFTWARE_START &&
           config.adc_trigger_edge != ADC_EXTERNALTRIGCONVEDGE_NONE;
  }

  return config.acquisition_mode ==
             stm32h5xx::cfg::adc_acquisition_mode::free_running &&
         config.adc_trigger == ADC_SOFTWARE_START;
}

void publish_frame(adc_pipeline_runtime& runtime,
                   const std::array<uint16_t, kAdcDmaMaxSequenceLength>& values,
                   const uint32_t samples_per_channel) noexcept {
  const auto channels = sequence_length(*runtime.p_config);
  runtime.frame.values.fill(0U);
  for (std::size_t index = 0U; index < channels; ++index) {
    runtime.frame.values[index] = values[index];
  }
  runtime.frame.channel_count = static_cast<uint16_t>(channels);
  runtime.frame.samples_per_channel = samples_per_channel;
  runtime.frame.frequency_hz = runtime.actual_frequency_hz;
  ++runtime.frame.sequence;
  runtime.frame_available = true;
  set_status(runtime, adc_status::OK);
}

void accumulate_dma_samples(adc_pipeline_runtime& runtime,
                            const std::size_t begin_sample,
                            const std::size_t end_sample) noexcept {
  const auto transfer = dma_transfer_length(*runtime.p_config);
  const auto begin = std::min(begin_sample, transfer);
  const auto end = std::min(end_sample, transfer);
  const auto channels = sequence_length(*runtime.p_config);

  for (std::size_t sample = begin; sample < end; ++sample) {
    const auto channel = sample % channels;
    runtime.average_sums[channel] += runtime.dma_buffer[sample];
    ++runtime.average_sample_counts[channel];
  }
}

void publish_average_frame(adc_pipeline_runtime& runtime) noexcept {
  std::array<uint16_t, kAdcDmaMaxSequenceLength> values{};
  const auto channels = sequence_length(*runtime.p_config);
  uint32_t samples{0U};
  for (std::size_t channel = 0U; channel < channels; ++channel) {
    const auto count = runtime.average_sample_counts[channel];
    if (count == 0U) {
      return;
    }
    values[channel] = static_cast<uint16_t>(
        runtime.average_sums[channel] / static_cast<uint64_t>(count));
    samples = count;
  }
  publish_frame(runtime, values, samples);
}

void accumulate_dma_until(adc_pipeline_runtime& runtime,
                          const std::size_t boundary) noexcept {
  if (boundary <= runtime.processed_samples_in_cycle) {
    return;
  }
  accumulate_dma_samples(runtime, runtime.processed_samples_in_cycle, boundary);
  runtime.processed_samples_in_cycle = boundary;
}

uint32_t channel_mask(const std::size_t index) noexcept {
  return 1UL << index;
}

void sum_fixed_dma_window(
    const adc_pipeline_runtime& runtime, const std::size_t end_sample,
    std::array<uint64_t, kAdcDmaMaxSequenceLength>& sums,
    std::array<uint32_t, kAdcDmaMaxSequenceLength>& counts) noexcept {
  const auto transfer = dma_transfer_length(*runtime.p_config);
  const auto width = window_width(*runtime.p_config);
  const auto channels = sequence_length(*runtime.p_config);
  for (std::size_t offset = 0U; offset < width; ++offset) {
    const auto sample = (end_sample + transfer - width + offset) % transfer;
    const auto channel = sample % channels;
    sums[channel] += runtime.dma_buffer[sample];
    ++counts[channel];
  }
}

void complete_fixed_dma_window(adc_pipeline_runtime& runtime,
                               const std::size_t boundary) noexcept {
  const auto transfer = dma_transfer_length(*runtime.p_config);
  if (boundary > transfer || boundary <= runtime.processed_samples_in_cycle) {
    return;
  }

  const auto width = window_width(*runtime.p_config);
  const auto completed = boundary - runtime.processed_samples_in_cycle;
  runtime.processed_samples_in_cycle = boundary;
  runtime.fixed_window_sample_count =
      std::min(width, runtime.fixed_window_sample_count + completed);
  if (runtime.fixed_window_sample_count < width) {
    return;
  }

  std::array<uint64_t, kAdcDmaMaxSequenceLength> sums{};
  std::array<uint32_t, kAdcDmaMaxSequenceLength> counts{};
  sum_fixed_dma_window(runtime, boundary, sums, counts);

  const auto channels = sequence_length(*runtime.p_config);
  uint32_t valid_channels{0U};
  for (std::size_t channel = 0U; channel < channels; ++channel) {
    if (counts[channel] == 0U) {
      continue;
    }
    runtime.fixed_window_values[channel] = static_cast<uint16_t>(
        sums[channel] / static_cast<uint64_t>(counts[channel]));
    valid_channels |= channel_mask(channel);
  }
  runtime.fixed_window_valid_channels = valid_channels;

  if (valid_channels == ((1UL << channels) - 1UL)) {
    publish_frame(runtime, runtime.fixed_window_values,
                  static_cast<uint32_t>(width / channels));
  }
}

bool prepare_dma_runtime(adc_pipeline_runtime& runtime) noexcept {
  if (runtime.p_config == nullptr || !valid_pipeline(*runtime.p_config)) {
    set_status(runtime, adc_status::INVALID_CONFIGURATION);
    return false;
  }

  runtime.dma_buffer = dma_buffer_for(runtime.p_config);
  runtime.dma_buffer_capacity = dma_buffer_capacity_for(runtime.p_config);
  runtime.average_sums.fill(0U);
  runtime.average_sample_counts.fill(0U);
  runtime.fixed_window_values.fill(0U);
  runtime.fixed_window_valid_channels = 0U;
  runtime.fixed_window_sample_count = 0U;
  runtime.processed_samples_in_cycle = 0U;
  runtime.frame = {};
  runtime.frame_available = false;
  return runtime.dma_buffer != nullptr &&
         dma_transfer_length(*runtime.p_config) <= runtime.dma_buffer_capacity;
}

HAL_StatusTypeDef configure_adc_channels(adc_pipeline_runtime& runtime) noexcept {
  const auto* const pipeline = runtime.p_config;
  if (pipeline == nullptr || pipeline->channel_sequence == nullptr) {
    return HAL_ERROR;
  }
  for (std::size_t index = 0U; index < pipeline->channel_sequence_length; ++index) {
    auto channel = pipeline->channel_sequence[index];
    const auto status = HAL_ADC_ConfigChannel(&runtime.m_handle, &channel);
    if (status != HAL_OK) {
      return status;
    }
  }
  return HAL_OK;
}

void configure_pipeline_pins(const adc_pipeline_runtime& runtime) noexcept {
#define RU_STM32H5XX_ADC_CONFIGURE_PIPELINE_PIN(name, config) \
  if (config.pipeline == runtime.p_config) {                  \
    init_pin(config.port(), config.pin_init);                 \
  }
  RU_STM32H5XX_ADC_MAP(RU_STM32H5XX_ADC_CONFIGURE_PIPELINE_PIN)
#undef RU_STM32H5XX_ADC_CONFIGURE_PIPELINE_PIN
}

bool configure_injected_channels(adc_pipeline_runtime& runtime) noexcept {
#define RU_STM32H5XX_ADC_CONFIGURE_INJECTED(name, config)                    \
  if (config.uses_injected && config.pipeline == runtime.p_config) {         \
    auto injected = config.injected_channel_init;                             \
    if (HAL_ADCEx_InjectedConfigChannel(&runtime.m_handle, &injected) !=      \
        HAL_OK) {                                                             \
      return false;                                                           \
    }                                                                          \
  }
  RU_STM32H5XX_ADC_MAP(RU_STM32H5XX_ADC_CONFIGURE_INJECTED)
#undef RU_STM32H5XX_ADC_CONFIGURE_INJECTED
  return true;
}

uint32_t timer_prescaler(const TIM_TypeDef* const instance,
                         const uint32_t counter_clock_hz) noexcept {
  const auto timer_clock_hz = timer_input_clock_hz(instance);
  return timer_clock_hz <= counter_clock_hz
             ? 0U
             : (timer_clock_hz / counter_clock_hz) - 1U;
}

uint32_t timer_period(const TIM_TypeDef* const instance,
                      const uint32_t counter_clock_hz,
                      const uint32_t frequency_hz) noexcept {
  const auto timer_clock_hz = timer_input_clock_hz(instance);
  const auto prescaler = timer_prescaler(instance, counter_clock_hz);
  const auto actual_counter_clock = timer_clock_hz / (prescaler + 1U);
  const auto ticks = std::max<uint32_t>(actual_counter_clock / frequency_hz, 1U);
  return ticks - 1U;
}

bool init_trigger_timer(adc_pipeline_runtime& runtime) noexcept {
  const auto* const config = runtime.p_config;
  if (config == nullptr || !config->uses_timer_trigger()) {
    return true;
  }

  enable_tim_clock(config->trigger_timer_instance());
  auto& handle = runtime.m_trigger_timer_handle;
  handle = {};
  handle.Instance = config->trigger_timer_instance();
  handle.Init = config->trigger_timer_init;
  handle.Init.Prescaler =
      timer_prescaler(handle.Instance, config->trigger_counter_clock_hz);
  handle.Init.Period = timer_period(handle.Instance,
                                    config->trigger_counter_clock_hz,
                                    config->trigger_frequency_hz);

  set_init_diagnostic(runtime, adc_init_stage::TRIGGER_TIMER_INIT);
  if (handle.Init.Period > timer_auto_reload_max(handle.Instance)) {
    handle.Instance = nullptr;
    set_init_diagnostic(
        runtime,
        adc_init_stage::TRIGGER_TIMER_INIT,
        adc_status::INVALID_CONFIGURATION);
    set_status(runtime, adc_status::INVALID_CONFIGURATION);
    return false;
  }
  const auto timer_status = HAL_TIM_Base_Init(&handle);
  if (timer_status != HAL_OK) {
    handle.Instance = nullptr;
    set_init_diagnostic(
        runtime,
        adc_init_stage::TRIGGER_TIMER_INIT,
        adc_status::HARDWARE_ERROR,
        static_cast<uint32_t>(timer_status));
    set_status(runtime, adc_status::HARDWARE_ERROR);
    return false;
  }

  set_init_diagnostic(runtime, adc_init_stage::TRIGGER_TIMER_CONFIG);
  auto master = config->trigger_master_config;
  const auto master_status =
      HAL_TIMEx_MasterConfigSynchronization(&handle, &master);
  if (master_status != HAL_OK) {
    (void)HAL_TIM_Base_DeInit(&handle);
    handle.Instance = nullptr;
    set_init_diagnostic(
        runtime,
        adc_init_stage::TRIGGER_TIMER_CONFIG,
        adc_status::HARDWARE_ERROR,
        static_cast<uint32_t>(master_status));
    set_status(runtime, adc_status::HARDWARE_ERROR);
    return false;
  }

  const auto timer_clock_hz = timer_input_clock_hz(handle.Instance);
  const auto counter_hz = timer_clock_hz / (handle.Init.Prescaler + 1U);
  runtime.actual_frequency_hz =
      counter_hz / (static_cast<uint32_t>(handle.Init.Period) + 1U);
  __HAL_TIM_SET_COUNTER(&handle, 0U);
  return true;
}

bool start_trigger_timer(adc_pipeline_runtime& runtime) noexcept {
  if (runtime.m_trigger_timer_handle.Instance == nullptr) {
    return true;
  }
  const auto timer_status = HAL_TIM_Base_Start(&runtime.m_trigger_timer_handle);
  if (timer_status != HAL_OK) {
    set_init_diagnostic(
        runtime,
        adc_init_stage::TRIGGER_TIMER_START,
        adc_status::HARDWARE_ERROR,
        static_cast<uint32_t>(timer_status));
    set_status(runtime, adc_status::HARDWARE_ERROR);
    return false;
  }
  return true;
}

ADC_InitTypeDef effective_adc_init(const pipeline_config& config) noexcept {
  auto init = config.init;
  switch (config.acquisition_mode) {
    case stm32h5xx::cfg::adc_acquisition_mode::polling:
      init.ContinuousConvMode = DISABLE;
      init.ExternalTrigConv = ADC_SOFTWARE_START;
      init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
      break;
    case stm32h5xx::cfg::adc_acquisition_mode::free_running:
      init.ContinuousConvMode = ENABLE;
      init.ExternalTrigConv = ADC_SOFTWARE_START;
      init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
      break;
    case stm32h5xx::cfg::adc_acquisition_mode::timer_triggered:
      init.ContinuousConvMode = DISABLE;
      init.ExternalTrigConv = config.adc_trigger;
      init.ExternalTrigConvEdge = config.adc_trigger_edge;
      break;
  }
  return init;
}

bool initialize_pipeline(adc_pipeline_runtime& runtime,
                         const pipeline_config* const config) noexcept {
  if (config == nullptr) {
    return false;
  }
  if (runtime.initialized) {
    return runtime.p_config == config;
  }
  if (runtime.p_config != nullptr && runtime.p_config != config) {
    initialize_diagnostic(runtime);
    set_init_diagnostic(
        runtime,
        adc_init_stage::VALIDATE_CONFIGURATION,
        adc_status::RESOURCE_CONFLICT);
    set_status(runtime, adc_status::RESOURCE_CONFLICT);
    return false;
  }
  runtime.p_config = config;
  initialize_diagnostic(runtime);
  set_init_diagnostic(runtime, adc_init_stage::VALIDATE_CONFIGURATION);

  if (!valid_pipeline(*config)) {
    set_init_diagnostic(
        runtime,
        adc_init_stage::VALIDATE_CONFIGURATION,
        adc_status::INVALID_CONFIGURATION);
    set_status(runtime, adc_status::INVALID_CONFIGURATION);
    return false;
  }

  if (config->uses_dma()) {
    set_init_diagnostic(runtime, adc_init_stage::PREPARE_DMA);
    if (!prepare_dma_runtime(runtime)) {
      set_init_diagnostic(
          runtime,
          adc_init_stage::PREPARE_DMA,
          adc_status::HARDWARE_ERROR);
      set_status(runtime, adc_status::HARDWARE_ERROR);
      return false;
    }
    __HAL_RCC_ADC_CLK_ENABLE();
    __HAL_RCC_GPDMA1_CLK_ENABLE();
    if (config->uses_timer_trigger() && !init_trigger_timer(runtime)) {
      return false;
    }

    DMA_NodeConfTypeDef node{};
    node.NodeType = DMA_GPDMA_LINEAR_NODE;
    node.Init.Request = config->dma_request;
    node.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
    node.Init.Direction = DMA_PERIPH_TO_MEMORY;
    node.Init.SrcInc = DMA_SINC_FIXED;
    node.Init.DestInc = DMA_DINC_INCREMENTED;
    node.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_HALFWORD;
    node.Init.DestDataWidth = DMA_DEST_DATAWIDTH_HALFWORD;
    node.Init.SrcBurstLength = 1U;
    node.Init.DestBurstLength = 1U;
    node.Init.TransferAllocatedPort =
        DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT0;
    node.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
    node.Init.Mode = DMA_NORMAL;
    node.TriggerConfig.TriggerPolarity = DMA_TRIG_POLARITY_MASKED;
    node.DataHandlingConfig.DataExchange = DMA_EXCHANGE_NONE;
    node.DataHandlingConfig.DataAlignment = DMA_DATA_RIGHTALIGN_ZEROPADDED;

    set_init_diagnostic(runtime, adc_init_stage::DMA_NODE_BUILD);
    const auto node_status =
        HAL_DMAEx_List_BuildNode(&node, &runtime.m_dma_node);
    if (node_status != HAL_OK) {
      set_init_diagnostic(
          runtime,
          adc_init_stage::DMA_NODE_BUILD,
          adc_status::HARDWARE_ERROR,
          static_cast<uint32_t>(node_status));
      set_status(runtime, adc_status::HARDWARE_ERROR);
      return false;
    }

    set_init_diagnostic(runtime, adc_init_stage::DMA_NODE_INSERT);
    const auto insert_status = HAL_DMAEx_List_InsertNode(
        &runtime.m_dma_queue, nullptr, &runtime.m_dma_node);
    if (insert_status != HAL_OK) {
      set_init_diagnostic(
          runtime,
          adc_init_stage::DMA_NODE_INSERT,
          adc_status::HARDWARE_ERROR,
          static_cast<uint32_t>(insert_status));
      set_status(runtime, adc_status::HARDWARE_ERROR);
      return false;
    }

    set_init_diagnostic(runtime, adc_init_stage::DMA_QUEUE_CIRCULAR);
    const auto circular_status =
        HAL_DMAEx_List_SetCircularMode(&runtime.m_dma_queue);
    if (circular_status != HAL_OK) {
      set_init_diagnostic(
          runtime,
          adc_init_stage::DMA_QUEUE_CIRCULAR,
          adc_status::HARDWARE_ERROR,
          static_cast<uint32_t>(circular_status));
      set_status(runtime, adc_status::HARDWARE_ERROR);
      return false;
    }

    runtime.m_dma_channel_handle.Instance = config->dma_channel();
    runtime.m_dma_channel_handle.InitLinkedList.Priority =
        DMA_LOW_PRIORITY_LOW_WEIGHT;
    runtime.m_dma_channel_handle.InitLinkedList.LinkStepMode =
        DMA_LSM_FULL_EXECUTION;
    runtime.m_dma_channel_handle.InitLinkedList.LinkAllocatedPort =
        DMA_LINK_ALLOCATED_PORT0;
    runtime.m_dma_channel_handle.InitLinkedList.TransferEventMode =
        DMA_TCEM_BLOCK_TRANSFER;
    runtime.m_dma_channel_handle.InitLinkedList.LinkedListMode =
        DMA_LINKEDLIST_CIRCULAR;

    set_init_diagnostic(runtime, adc_init_stage::DMA_CHANNEL_INIT);
    const auto dma_init_status =
        HAL_DMAEx_List_Init(&runtime.m_dma_channel_handle);
    if (dma_init_status != HAL_OK) {
      set_init_diagnostic(
          runtime,
          adc_init_stage::DMA_CHANNEL_INIT,
          adc_status::HARDWARE_ERROR,
          static_cast<uint32_t>(dma_init_status));
      set_status(runtime, adc_status::HARDWARE_ERROR);
      return false;
    }

    set_init_diagnostic(runtime, adc_init_stage::DMA_QUEUE_LINK);
    const auto link_status = HAL_DMAEx_List_LinkQ(
        &runtime.m_dma_channel_handle, &runtime.m_dma_queue);
    if (link_status != HAL_OK) {
      set_init_diagnostic(
          runtime,
          adc_init_stage::DMA_QUEUE_LINK,
          adc_status::HARDWARE_ERROR,
          static_cast<uint32_t>(link_status));
      set_status(runtime, adc_status::HARDWARE_ERROR);
      return false;
    }

    set_init_diagnostic(runtime, adc_init_stage::DMA_CHANNEL_ATTRIBUTES);
    const auto attributes_status = HAL_DMA_ConfigChannelAttributes(
        &runtime.m_dma_channel_handle, DMA_CHANNEL_NPRIV);
    if (attributes_status != HAL_OK) {
      set_init_diagnostic(
          runtime,
          adc_init_stage::DMA_CHANNEL_ATTRIBUTES,
          adc_status::HARDWARE_ERROR,
          static_cast<uint32_t>(attributes_status));
      set_status(runtime, adc_status::HARDWARE_ERROR);
      return false;
    }

    HAL_NVIC_SetPriority(config->dma_irq, 6U, 0U);
    HAL_NVIC_EnableIRQ(config->dma_irq);
  } else {
    __HAL_RCC_ADC_CLK_ENABLE();
  }

  runtime.m_handle.Instance = config->instance();
  runtime.m_handle.Init = effective_adc_init(*config);
  configure_pipeline_pins(runtime);
  if (config->uses_dma()) {
    __HAL_LINKDMA(&runtime.m_handle, DMA_Handle,
                  runtime.m_dma_channel_handle);
  }

  set_init_diagnostic(runtime, adc_init_stage::ADC_INIT);
  const auto adc_init_status = HAL_ADC_Init(&runtime.m_handle);
  if (adc_init_status != HAL_OK) {
    set_init_diagnostic(
        runtime,
        adc_init_stage::ADC_INIT,
        adc_status::HARDWARE_ERROR,
        static_cast<uint32_t>(adc_init_status));
    set_status(runtime, adc_status::HARDWARE_ERROR);
    return false;
  }

  set_init_diagnostic(runtime, adc_init_stage::CHANNEL_CONFIG);
  const auto channel_status = configure_adc_channels(runtime);
  if (channel_status != HAL_OK) {
    set_init_diagnostic(
        runtime,
        adc_init_stage::CHANNEL_CONFIG,
        adc_status::HARDWARE_ERROR,
        static_cast<uint32_t>(channel_status));
    set_status(runtime, adc_status::HARDWARE_ERROR);
    return false;
  }

  set_init_diagnostic(runtime, adc_init_stage::CALIBRATION);
  const auto calibration_status =
      HAL_ADCEx_Calibration_Start(&runtime.m_handle, ADC_SINGLE_ENDED);
  if (calibration_status != HAL_OK) {
    set_init_diagnostic(
        runtime,
        adc_init_stage::CALIBRATION,
        adc_status::HARDWARE_ERROR,
        static_cast<uint32_t>(calibration_status));
    set_status(runtime, adc_status::HARDWARE_ERROR);
    return false;
  }

  set_init_diagnostic(runtime, adc_init_stage::INJECTED_CHANNEL_CONFIG);
  if (!configure_injected_channels(runtime)) {
    set_init_diagnostic(
        runtime,
        adc_init_stage::INJECTED_CHANNEL_CONFIG,
        adc_status::HARDWARE_ERROR,
        static_cast<uint32_t>(HAL_ERROR));
    set_status(runtime, adc_status::HARDWARE_ERROR);
    return false;
  }

  if (config->uses_dma()) {
    std::fill_n(runtime.dma_buffer, dma_transfer_length(*config), 0U);
    set_init_diagnostic(runtime, adc_init_stage::DMA_START);
    const auto dma_start_status = HAL_ADC_Start_DMA(
        &runtime.m_handle, reinterpret_cast<uint32_t*>(runtime.dma_buffer),
        static_cast<uint32_t>(dma_transfer_length(*config)));
    if (dma_start_status != HAL_OK) {
      set_init_diagnostic(
          runtime,
          adc_init_stage::DMA_START,
          adc_status::HARDWARE_ERROR,
          static_cast<uint32_t>(dma_start_status));
      set_status(runtime, adc_status::HARDWARE_ERROR);
      return false;
    }

    set_init_diagnostic(runtime, adc_init_stage::TRIGGER_TIMER_START);
    if (!start_trigger_timer(runtime)) {
      return false;
    }
  }

  runtime.initialized = true;
  set_status(runtime, adc_status::OK);
  set_init_diagnostic(runtime, adc_init_stage::READY, adc_status::OK);
  return true;
}

expected::expected<uint16_t, result> read_dma_average(
    const opaque_adc& config) noexcept {
  auto* const runtime = runtime_for_config(config.m_p_config);
  if (runtime == nullptr || !runtime->initialized) {
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }
  const auto index = config.dma_sequence_index();
  if (index >= sequence_length(*runtime->p_config)) {
    return expected::unexpected(result::UNRECOVERABLE_ERROR);
  }
  const auto primask = lock_irq();
  const auto sum = runtime->average_sums[index];
  const auto count = runtime->average_sample_counts[index];
  runtime->average_sums[index] = 0U;
  runtime->average_sample_counts[index] = 0U;
  unlock_irq(primask);
  if (count == 0U) {
    set_status(*runtime, adc_status::NOT_READY);
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }
  return expected::expected<uint16_t, result>(static_cast<uint16_t>(sum / count));
}

expected::expected<std::optional<uint16_t>, result> try_read_dma_average(
    const opaque_adc& config) noexcept {
  auto* const runtime = runtime_for_config(config.m_p_config);
  if (runtime == nullptr || !runtime->initialized) {
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }
  const auto index = config.dma_sequence_index();
  if (index >= sequence_length(*runtime->p_config)) {
    return expected::unexpected(result::UNRECOVERABLE_ERROR);
  }
  const auto primask = lock_irq();
  const auto sum = runtime->average_sums[index];
  const auto count = runtime->average_sample_counts[index];
  runtime->average_sums[index] = 0U;
  runtime->average_sample_counts[index] = 0U;
  unlock_irq(primask);
  if (count == 0U) {
    set_status(*runtime, adc_status::NOT_READY);
  }
  return count == 0U ? std::optional<uint16_t>{}
                     : std::optional<uint16_t>{static_cast<uint16_t>(sum / count)};
}

expected::expected<uint16_t, result> read_dma_fixed_window(
    const opaque_adc& config) noexcept {
  auto* const runtime = runtime_for_config(config.m_p_config);
  if (runtime == nullptr || !runtime->initialized) {
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }
  const auto index = config.dma_sequence_index();
  if (index >= sequence_length(*runtime->p_config)) {
    return expected::unexpected(result::UNRECOVERABLE_ERROR);
  }
  const auto primask = lock_irq();
  const auto valid = (runtime->fixed_window_valid_channels & channel_mask(index)) != 0U;
  const auto value = runtime->fixed_window_values[index];
  unlock_irq(primask);
  if (!valid) {
    set_status(*runtime, adc_status::NOT_READY);
  }
  return valid ? expected::expected<uint16_t, result>(value)
               : expected::unexpected(result::RECOVERABLE_ERROR);
}

expected::expected<std::optional<uint16_t>, result> try_read_dma_fixed_window(
    const opaque_adc& config) noexcept {
  auto* const runtime = runtime_for_config(config.m_p_config);
  if (runtime == nullptr || !runtime->initialized) {
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }
  const auto index = config.dma_sequence_index();
  if (index >= sequence_length(*runtime->p_config)) {
    return expected::unexpected(result::UNRECOVERABLE_ERROR);
  }
  const auto primask = lock_irq();
  const auto valid = (runtime->fixed_window_valid_channels & channel_mask(index)) != 0U;
  const auto value = runtime->fixed_window_values[index];
  unlock_irq(primask);
  if (!valid) {
    set_status(*runtime, adc_status::NOT_READY);
  }
  return valid ? std::optional<uint16_t>{value} : std::optional<uint16_t>{};
}

expected::expected<uint16_t, result> read_injected_conversion(
    const opaque_adc& config) noexcept {
  auto* const runtime = runtime_for_config(config.m_p_config);
  if (runtime == nullptr || !runtime->initialized || !config.uses_injected()) {
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }
  if (HAL_ADCEx_InjectedStart(&runtime->m_handle) != HAL_OK ||
      HAL_ADCEx_InjectedPollForConversion(&runtime->m_handle,
                                          config.polling_timeout_ms()) != HAL_OK) {
    (void)HAL_ADCEx_InjectedStop(&runtime->m_handle);
    set_status(*runtime, adc_status::TIMEOUT);
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }
  const auto value = static_cast<uint16_t>(HAL_ADCEx_InjectedGetValue(
      &runtime->m_handle, config.injected_channel_init().InjectedRank));
  (void)HAL_ADCEx_InjectedStop(&runtime->m_handle);
  return value;
}

expected::expected<std::optional<uint16_t>, result> try_read_injected_conversion(
    const opaque_adc& config) noexcept {
  auto* const runtime = runtime_for_config(config.m_p_config);
  if (runtime == nullptr || !runtime->initialized || !config.uses_injected()) {
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }
  if (HAL_ADCEx_InjectedStart(&runtime->m_handle) != HAL_OK) {
    set_status(*runtime, adc_status::HARDWARE_ERROR);
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }
  const auto poll = HAL_ADCEx_InjectedPollForConversion(&runtime->m_handle, 0U);
  if (poll == HAL_TIMEOUT) {
    (void)HAL_ADCEx_InjectedStop(&runtime->m_handle);
    return std::optional<uint16_t>{};
  }
  if (poll != HAL_OK) {
    (void)HAL_ADCEx_InjectedStop(&runtime->m_handle);
    set_status(*runtime, adc_status::HARDWARE_ERROR);
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }
  const auto value = static_cast<uint16_t>(HAL_ADCEx_InjectedGetValue(
      &runtime->m_handle, config.injected_channel_init().InjectedRank));
  (void)HAL_ADCEx_InjectedStop(&runtime->m_handle);
  return std::optional<uint16_t>{value};
}

}  // namespace

result Adc::start() noexcept {
  opaque_adc::start();
  return result::OK;
}

Adc::Adc(const AdcId id) noexcept : m_id(id), m_opaque(make_opaque(id)) {}

result Adc::init() noexcept {
  return m_opaque.init();
}

result Adc::stop() noexcept {
  return m_opaque.stop();
}

expected::expected<uint16_t, result> Adc::read() noexcept {
  uint16_t value{0U};
  const auto status = m_opaque.read(value);
  if (status != result::OK) {
    return expected::unexpected(status);
  }
  return value;
}

expected::expected<std::optional<uint16_t>, result> Adc::try_read() noexcept {
  bool has_value{false};
  uint16_t value{0U};
  const auto status = m_opaque.try_read(has_value, value);
  if (status != result::OK) {
    return expected::unexpected(status);
  }
  return has_value ? std::optional<uint16_t>{value} : std::optional<uint16_t>{};
}

expected::expected<adc_frame, adc_status> Adc::snapshot() noexcept {
  adc_frame frame{};
  const auto status = m_opaque.snapshot(frame);
  return status == adc_status::OK ? expected::expected<adc_frame, adc_status>(frame)
                                  : expected::unexpected(status);
}

expected::expected<std::optional<adc_frame>, adc_status> Adc::try_snapshot() noexcept {
  adc_frame frame{};
  bool has_frame{false};
  const auto status = m_opaque.try_snapshot(has_frame, frame);
  if (status != adc_status::OK) {
    return expected::unexpected(status);
  }
  return has_frame ? std::optional<adc_frame>{frame}
                   : std::optional<adc_frame>{};
}

adc_status Adc::status() const noexcept {
  return m_opaque.status();
}

adc_init_diagnostic Adc::diagnostics() const noexcept {
  return m_opaque.diagnostics();
}

uint16_t Adc::sample_max() const noexcept {
  return m_opaque.sample_max();
}

void adc_dma_half_transfer_callback(ADC_HandleTypeDef* const handle) noexcept {
  auto* const runtime = runtime_from_handle(handle);
  if (runtime == nullptr || !runtime->initialized || runtime->p_config == nullptr) {
    return;
  }
  const auto half = dma_transfer_length(*runtime->p_config) / 2U;
  if (uses_fixed_window_average(*runtime)) {
    complete_fixed_dma_window(*runtime, half);
  } else {
    accumulate_dma_until(*runtime, half);
  }
}

void adc_dma_full_transfer_callback(ADC_HandleTypeDef* const handle) noexcept {
  auto* const runtime = runtime_from_handle(handle);
  if (runtime == nullptr || !runtime->initialized || runtime->p_config == nullptr) {
    return;
  }
  const auto transfer = dma_transfer_length(*runtime->p_config);
  if (uses_fixed_window_average(*runtime)) {
    complete_fixed_dma_window(*runtime, transfer);
  } else {
    accumulate_dma_until(*runtime, transfer);
    publish_average_frame(*runtime);
  }
  runtime->processed_samples_in_cycle = 0U;
}

void adc_dma_irq_handler(ADC_TypeDef* const instance) noexcept {
  auto* const runtime = runtime_for_instance(instance);
  if (runtime != nullptr && runtime->m_dma_channel_handle.Instance != nullptr) {
    HAL_DMA_IRQHandler(&runtime->m_dma_channel_handle);
  }
}

bool opaque_adc::initialized() const noexcept {
  return m_p_runtime != nullptr && m_p_runtime->initialized &&
         m_p_runtime->p_config ==
             (m_p_config != nullptr ? m_p_config->pipeline : nullptr);
}

std::optional<AdcId> opaque_adc::id() const noexcept {
  for (std::size_t index = 0U; index < static_cast<std::size_t>(AdcId::COUNT);
       ++index) {
    const auto id = static_cast<AdcId>(index);
    if (config_for(id) == m_p_config) {
      return id;
    }
  }
  return std::nullopt;
}

bool opaque_adc::uses_dma() const noexcept {
  return m_p_config != nullptr && m_p_config->pipeline != nullptr &&
         m_p_config->pipeline->uses_dma();
}

bool opaque_adc::uses_injected() const noexcept {
  return m_p_config != nullptr && m_p_config->uses_injected;
}

const pipeline_config* opaque_adc::pipeline() const noexcept {
  return m_p_config != nullptr ? m_p_config->pipeline : nullptr;
}

ADC_TypeDef* opaque_adc::instance() const noexcept {
  return pipeline() != nullptr ? pipeline()->instance() : nullptr;
}

GPIO_TypeDef* opaque_adc::port() const noexcept {
  return m_p_config != nullptr ? m_p_config->port() : nullptr;
}

const GPIO_InitTypeDef& opaque_adc::pin_init() const noexcept {
  return m_p_config->pin_init;
}

const ADC_InitTypeDef& opaque_adc::adc_init() const noexcept {
  return pipeline()->init;
}

const ADC_ChannelConfTypeDef& opaque_adc::channel_init() const noexcept {
  return m_p_config->channel_init;
}

const ADC_InjectionConfTypeDef& opaque_adc::injected_channel_init() const noexcept {
  return m_p_config->injected_channel_init;
}

uint32_t opaque_adc::polling_timeout_ms() const noexcept {
  return m_p_config != nullptr ? m_p_config->polling_timeout_ms
                               : stm32h5xx::cfg::kDefaultAdcPollTimeoutMs;
}

uint16_t opaque_adc::sample_max() const noexcept {
  return m_p_config != nullptr ? m_p_config->sample_max
                               : stm32h5xx::cfg::kDefaultAdcSampleMax;
}

const ADC_ChannelConfTypeDef* opaque_adc::dma_channel_sequence() const noexcept {
  return pipeline() != nullptr ? pipeline()->channel_sequence : nullptr;
}

std::size_t opaque_adc::dma_frame_count() const noexcept {
  return pipeline() != nullptr ? pipeline()->dma_frame_count : 0U;
}

std::size_t opaque_adc::dma_sequence_length() const noexcept {
  return pipeline() != nullptr ? pipeline()->channel_sequence_length : 0U;
}

std::size_t opaque_adc::dma_sequence_index() const noexcept {
  return m_p_config != nullptr ? m_p_config->sequence_index : 0U;
}

stm32h5xx::cfg::adc_dma_backend opaque_adc::dma_backend() const noexcept {
  return pipeline() != nullptr
             ? pipeline()->dma_backend
             : stm32h5xx::cfg::adc_dma_backend::average_since_read;
}

std::size_t opaque_adc::dma_window_width() const noexcept {
  return pipeline() != nullptr ? pipeline()->dma_window_width : 0U;
}

uint32_t opaque_adc::dma_request() const noexcept {
  return pipeline() != nullptr ? pipeline()->dma_request : 0U;
}

DMA_Channel_TypeDef* opaque_adc::dma_channel() const noexcept {
  return pipeline() != nullptr ? pipeline()->dma_channel() : nullptr;
}

IRQn_Type opaque_adc::dma_irq() const noexcept {
  return pipeline() != nullptr ? pipeline()->dma_irq : static_cast<IRQn_Type>(0);
}

bool opaque_adc::uses_timer_trigger() const noexcept {
  return pipeline() != nullptr && pipeline()->uses_timer_trigger();
}

TIM_TypeDef* opaque_adc::trigger_timer_instance() const noexcept {
  return pipeline() != nullptr ? pipeline()->trigger_timer_instance() : nullptr;
}

uint32_t opaque_adc::trigger_counter_clock_hz() const noexcept {
  return pipeline() != nullptr ? pipeline()->trigger_counter_clock_hz : 0U;
}

uint32_t opaque_adc::trigger_frequency_hz() const noexcept {
  return pipeline() != nullptr ? pipeline()->trigger_frequency_hz : 0U;
}

const TIM_Base_InitTypeDef& opaque_adc::trigger_timer_init() const noexcept {
  return pipeline()->trigger_timer_init;
}

const TIM_MasterConfigTypeDef& opaque_adc::trigger_master_config() const noexcept {
  return pipeline()->trigger_master_config;
}

void opaque_adc::start() noexcept {
  __HAL_RCC_ADC_CLK_ENABLE();
}

result opaque_adc::init() noexcept {
  if (!valid() || pipeline() == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  init_pin(port(), pin_init());
  auto* const runtime = runtime_for_config(m_p_config);
  if (runtime == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }
  m_p_runtime = runtime;

  if (runtime->p_config != nullptr && runtime->p_config != pipeline()) {
    initialize_diagnostic(*runtime);
    set_init_diagnostic(
        *runtime,
        adc_init_stage::VALIDATE_CONFIGURATION,
        adc_status::RESOURCE_CONFLICT);
    set_status(*runtime, adc_status::RESOURCE_CONFLICT);
    return result::UNRECOVERABLE_ERROR;
  }
  if (initialized()) {
    return result::OK;
  }
  const auto initialized_pipeline = initialize_pipeline(*runtime, pipeline());
  if (initialized_pipeline) {
    return result::OK;
  }
  return runtime->last_status == adc_status::INVALID_CONFIGURATION ||
                 runtime->last_status == adc_status::RESOURCE_CONFLICT
             ? result::UNRECOVERABLE_ERROR
             : result::RECOVERABLE_ERROR;
}

result opaque_adc::stop() noexcept {
  auto* const runtime = m_p_runtime != nullptr ? m_p_runtime
                                               : runtime_for_config(m_p_config);
  if (runtime == nullptr || !runtime->initialized) {
    return result::OK;
  }

  result status = result::OK;
  if (runtime->m_trigger_timer_handle.Instance != nullptr) {
    if (HAL_TIM_Base_Stop(&runtime->m_trigger_timer_handle) != HAL_OK ||
        HAL_TIM_Base_DeInit(&runtime->m_trigger_timer_handle) != HAL_OK) {
      status = result::RECOVERABLE_ERROR;
    }
  }
  if (uses_dma() && HAL_ADC_Stop_DMA(&runtime->m_handle) != HAL_OK) {
    status = result::RECOVERABLE_ERROR;
  }
  if (HAL_ADC_DeInit(&runtime->m_handle) != HAL_OK) {
    status = result::RECOVERABLE_ERROR;
  }
  runtime->m_handle = {};
  runtime->m_dma_channel_handle = {};
  runtime->m_trigger_timer_handle = {};
  runtime->initialized = false;
  runtime->frame_available = false;
  runtime->last_status = adc_status::NOT_INITIALIZED;
  m_p_runtime = nullptr;
  return status;
}

result opaque_adc::read(uint16_t& r_value) noexcept {
  if (!initialized()) {
    return result::RECOVERABLE_ERROR;
  }
  if (uses_dma()) {
    const auto value = uses_fixed_window_average(*m_p_runtime)
                           ? read_dma_fixed_window(*this)
                           : read_dma_average(*this);
    if (!value.has_value()) {
      return value.error();
    }
    r_value = *value;
    return result::OK;
  }
  if (uses_injected()) {
    const auto value = read_injected_conversion(*this);
    if (!value.has_value()) {
      return value.error();
    }
    r_value = *value;
    return result::OK;
  }
  if (HAL_ADC_Start(&m_p_runtime->m_handle) != HAL_OK ||
      HAL_ADC_PollForConversion(&m_p_runtime->m_handle, polling_timeout_ms()) !=
          HAL_OK) {
    (void)HAL_ADC_Stop(&m_p_runtime->m_handle);
    set_status(*m_p_runtime, adc_status::TIMEOUT);
    return result::RECOVERABLE_ERROR;
  }
  r_value = static_cast<uint16_t>(HAL_ADC_GetValue(&m_p_runtime->m_handle));
  (void)HAL_ADC_Stop(&m_p_runtime->m_handle);
  return result::OK;
}

result opaque_adc::try_read(bool& r_has_value, uint16_t& r_value) noexcept {
  r_has_value = false;
  if (!initialized()) {
    return result::RECOVERABLE_ERROR;
  }
  if (uses_dma()) {
    const auto value = uses_fixed_window_average(*m_p_runtime)
                           ? try_read_dma_fixed_window(*this)
                           : try_read_dma_average(*this);
    if (!value.has_value()) {
      return value.error();
    }
    if (!value->has_value()) {
      return result::OK;
    }
    r_has_value = true;
    r_value = **value;
    return result::OK;
  }
  if (uses_injected()) {
    const auto value = try_read_injected_conversion(*this);
    if (!value.has_value()) {
      return value.error();
    }
    if (!value->has_value()) {
      return result::OK;
    }
    r_has_value = true;
    r_value = **value;
    return result::OK;
  }
  if (HAL_ADC_Start(&m_p_runtime->m_handle) != HAL_OK) {
    return result::RECOVERABLE_ERROR;
  }
  const auto poll = HAL_ADC_PollForConversion(&m_p_runtime->m_handle, 0U);
  if (poll == HAL_TIMEOUT) {
    (void)HAL_ADC_Stop(&m_p_runtime->m_handle);
    return result::OK;
  }
  if (poll != HAL_OK) {
    (void)HAL_ADC_Stop(&m_p_runtime->m_handle);
    return result::RECOVERABLE_ERROR;
  }
  r_has_value = true;
  r_value = static_cast<uint16_t>(HAL_ADC_GetValue(&m_p_runtime->m_handle));
  (void)HAL_ADC_Stop(&m_p_runtime->m_handle);
  return result::OK;
}

adc_status opaque_adc::snapshot(adc_frame& r_frame) noexcept {
  if (!initialized()) {
    return adc_status::NOT_INITIALIZED;
  }
  if (uses_dma()) {
    const auto primask = lock_irq();
    if (!m_p_runtime->frame_available) {
      unlock_irq(primask);
      set_status(*m_p_runtime, adc_status::NOT_READY);
      return adc_status::NOT_READY;
    }
    r_frame = m_p_runtime->frame;
    unlock_irq(primask);
    return adc_status::OK;
  }

  uint16_t value{0U};
  const auto read_status = read(value);
  if (read_status != result::OK) {
    return read_status == result::RECOVERABLE_ERROR ? adc_status::TIMEOUT
                                                     : adc_status::HARDWARE_ERROR;
  }
  r_frame = {};
  r_frame.sequence = ++m_p_runtime->frame.sequence;
  r_frame.samples_per_channel = 1U;
  r_frame.channel_count = 1U;
  r_frame.values[0] = value;
  return adc_status::OK;
}

adc_status opaque_adc::try_snapshot(bool& r_has_frame,
                                    adc_frame& r_frame) noexcept {
  r_has_frame = false;
  if (!initialized()) {
    return adc_status::NOT_INITIALIZED;
  }
  if (uses_dma()) {
    const auto primask = lock_irq();
    if (!m_p_runtime->frame_available) {
      unlock_irq(primask);
      return adc_status::OK;
    }
    r_frame = m_p_runtime->frame;
    unlock_irq(primask);
    r_has_frame = true;
    return adc_status::OK;
  }

  uint16_t value{0U};
  bool has_value{false};
  result read_status{result::RECOVERABLE_ERROR};
  if (uses_injected()) {
    const auto injected = try_read_injected_conversion(*this);
    if (!injected.has_value()) {
      return adc_status::HARDWARE_ERROR;
    }
    if (!injected->has_value()) {
      return adc_status::OK;
    }
    value = **injected;
    has_value = true;
    read_status = result::OK;
  } else {
    read_status = try_read(has_value, value);
  }
  if (read_status != result::OK) {
    return adc_status::HARDWARE_ERROR;
  }
  if (!has_value) {
    return adc_status::OK;
  }

  r_frame = {};
  r_frame.sequence = ++m_p_runtime->frame.sequence;
  r_frame.samples_per_channel = 1U;
  r_frame.channel_count = 1U;
  r_frame.values[0] = value;
  r_has_frame = true;
  return adc_status::OK;
}

adc_status opaque_adc::status() const noexcept {
  return m_p_runtime != nullptr ? m_p_runtime->last_status
                                : adc_status::NOT_INITIALIZED;
}

adc_init_diagnostic opaque_adc::diagnostics() const noexcept {
  return m_p_runtime != nullptr ? m_p_runtime->init_diagnostic
                                : adc_init_diagnostic{};
}

}  // namespace ru::driver

extern "C" void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* const handle) {
  ru::driver::adc_dma_half_transfer_callback(handle);
}

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* const handle) {
  ru::driver::adc_dma_full_transfer_callback(handle);
}

extern "C" void GPDMA1_Channel0_IRQHandler(void) {
  ru::driver::adc_dma_irq_handler(ADC1);
}

extern "C" void GPDMA1_Channel1_IRQHandler(void) {
  ru::driver::adc_dma_irq_handler(ADC2);
}
