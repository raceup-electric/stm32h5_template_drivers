#include <algorithm>
#include <array>

#include "adc.hpp"
#include "mapping.hpp"
#include "opaque_adc.hpp"
#include "stm_common.hpp"
#include "stm32h5xx_hal_adc_ex.h"
#include "stm32h5xx_hal_dma_ex.h"
#include "stm32h5xx_hal_tim.h"
#include "stm32h5xx_hal_tim_ex.h"

using namespace ru::driver;

namespace ru::driver {
namespace {
constexpr std::size_t adc_index(const AdcId id) noexcept {
  return static_cast<std::size_t>(id);
}

uint32_t timer_prescaler(const TIM_TypeDef* instance,
                         const uint32_t counter_clock_hz) noexcept {
  const auto timer_clock_hz = timer_input_clock_hz(instance);
  const auto target_counter_clock_hz = counter_clock_hz == 0U ? 1U : counter_clock_hz;
  return timer_clock_hz > target_counter_clock_hz
             ? (timer_clock_hz / target_counter_clock_hz) - 1U
             : 0U;
}

uint32_t timer_period(const uint32_t counter_clock_hz,
                      const uint32_t frequency_hz) noexcept {
  const auto target_counter_clock_hz = counter_clock_hz == 0U ? 1U : counter_clock_hz;
  const auto target_frequency_hz = frequency_hz == 0U ? 1U : frequency_hz;
  const auto period_ticks =
      std::max<uint32_t>(target_counter_clock_hz / target_frequency_hz, 1U);
  return period_ticks - 1U;
}

ADC_HandleTypeDef& adc_handle(const AdcId id) noexcept {
  static std::array<ADC_HandleTypeDef, stm32h5xx::cfg::adc::COUNT> handles{};
  static ADC_HandleTypeDef dummy_handle{};
  if constexpr (stm32h5xx::cfg::adc::COUNT == 0U) {
    (void)id;
    return dummy_handle;
  }

  const auto index = adc_index(id);
  return index < handles.size() ? handles[index] : handles.front();
}

opaque_adc make_opaque(const AdcId id) noexcept {
  return opaque_adc{&adc_handle(id)};
}

struct dma_adc_backend {
  ADC_HandleTypeDef adc_handle{};
  TIM_HandleTypeDef trigger_timer_handle{};
  DMA_NodeTypeDef dma_node{};
  DMA_QListTypeDef dma_queue{};
  DMA_HandleTypeDef dma_channel_handle{};
  std::array<uint16_t, stm32h5xx::cfg::adc::DMA_BUFFER_CAPACITY> dma_buffer{};
  std::array<AdcId, stm32h5xx::cfg::adc::COUNT> ids{};
  std::array<uint64_t, stm32h5xx::cfg::adc::COUNT> sums{};
  std::array<uint32_t, stm32h5xx::cfg::adc::COUNT> sample_counts{};
  std::size_t channel_count{0U};
  std::size_t active_buffer_length{0U};
  std::size_t processed_samples_in_cycle{0U};
  bool init_attempted{false};
  bool running{false};
};

std::array<dma_adc_backend, stm32h5xx::cfg::adc::DMA_BACKEND_COUNT>&
dma_backends() noexcept {
  static std::array<dma_adc_backend, stm32h5xx::cfg::adc::DMA_BACKEND_COUNT>
      backends{};
  return backends;
}

const stm32h5xx::cfg::adc::named_config* dma_adc_config(const AdcId id) noexcept {
  const auto* const config = stm32h5xx::cfg::adc::config_for(id);
  if (config == nullptr || !config->uses_dma()) {
    return nullptr;
  }

  for (const auto& named_config : stm32h5xx::cfg::adc::CONFIGS) {
    if (named_config.id == id) {
      return &named_config;
    }
  }

  return nullptr;
}

dma_adc_backend* dma_backend_for_instance(ADC_TypeDef* const p_instance) noexcept {
  auto& backends = dma_backends();
  for (std::size_t index = 0U; index < stm32h5xx::cfg::adc::CONFIGS.size(); ++index) {
    if (!stm32h5xx::cfg::adc::CONFIGS[index].config.uses_dma()) {
      continue;
    }

    if (stm32h5xx::cfg::adc::CONFIGS[index].config.instance() == p_instance) {
      return &backends[index];
    }
  }

  return nullptr;
}

dma_adc_backend* running_dma_backend_for(const AdcId id) noexcept {
  const auto* const config = dma_adc_config(id);
  if (config == nullptr) {
    return nullptr;
  }

  auto* const backend = dma_backend_for_instance(config->config.instance());
  if (backend == nullptr || !backend->running) {
    return nullptr;
  }

  return backend;
}

dma_adc_backend* dma_backend_from_handle(ADC_HandleTypeDef* const hadc) noexcept {
  if (hadc == nullptr) {
    return nullptr;
  }

  for (auto& backend : dma_backends()) {
    if (&backend.adc_handle == hadc) {
      return &backend;
    }
  }

  return nullptr;
}

void accumulate_dma_samples(dma_adc_backend& backend, const std::size_t begin_sample,
                            const std::size_t end_sample) noexcept {
  if (backend.channel_count == 0U || backend.active_buffer_length == 0U) {
    (void)begin_sample;
    (void)end_sample;
    return;
  }

  const std::size_t begin = std::min(begin_sample, backend.active_buffer_length);
  const std::size_t end = std::min(end_sample, backend.active_buffer_length);

  for (std::size_t sample = begin; sample < end; ++sample) {
    const auto rank = sample % backend.channel_count;
    const auto channel = adc_index(backend.ids[rank]);
    if (channel < stm32h5xx::cfg::adc::COUNT) {
      backend.sums[channel] += backend.dma_buffer[sample];
      backend.sample_counts[channel] += 1U;
    }
  }
}

void accumulate_dma_until(dma_adc_backend& backend, const std::size_t boundary) noexcept {
  if (boundary <= backend.processed_samples_in_cycle) {
    return;
  }

  accumulate_dma_samples(backend, backend.processed_samples_in_cycle, boundary);
  backend.processed_samples_in_cycle = boundary;
}

bool prepare_adc_channels(dma_adc_backend& backend,
                          ADC_TypeDef* const p_instance,
                          const std::size_t frame_count) noexcept {
  std::size_t rank = 0U;
  for (const auto& config : stm32h5xx::cfg::adc::CONFIGS) {
    if (!config.config.uses_dma() || config.config.instance() != p_instance) {
      continue;
    }

    if (rank >= backend.ids.size()) {
      return false;
    }

    init_pin(config.config.port(), config.config.pin_init);
    backend.ids[rank] = config.id;
    ++rank;
  }

  backend.channel_count = rank;
  backend.active_buffer_length =
      backend.channel_count * (frame_count == 0U ? 1U : frame_count);
  return backend.channel_count != 0U &&
         backend.active_buffer_length <= backend.dma_buffer.size();
}

bool configure_adc_channels(dma_adc_backend& backend,
                            ADC_TypeDef* const p_instance) noexcept {
  std::size_t rank = 0U;
  for (const auto& config : stm32h5xx::cfg::adc::CONFIGS) {
    if (!config.config.uses_dma() || config.config.instance() != p_instance) {
      continue;
    }

    if (rank >= backend.channel_count) {
      return false;
    }

    const auto* const dma = config.config.dma();
    if (dma == nullptr) {
      return false;
    }

    auto channel_config = dma->channel_init;
    channel_config.Rank = static_cast<uint32_t>(rank + 1U);

    if (HAL_ADC_ConfigChannel(&backend.adc_handle, &channel_config) != HAL_OK) {
      return false;
    }

    ++rank;
  }

  return rank == backend.channel_count;
}

bool init_trigger_timer(
    dma_adc_backend& backend,
    const stm32h5xx::cfg::adc_config::dma_backend_config::timer_trigger_config&
        trigger) noexcept {
  enable_tim_clock(trigger.timer_instance());

  auto& timer_handle = backend.trigger_timer_handle;
  timer_handle = {};
  timer_handle.Instance = trigger.timer_instance();
  timer_handle.Init = trigger.timer_init;
  timer_handle.Init.Prescaler =
      timer_prescaler(trigger.timer_instance(), trigger.counter_clock_hz);
  timer_handle.Init.Period =
      timer_period(trigger.counter_clock_hz, trigger.frequency_hz);

  if (HAL_TIM_Base_Init(&timer_handle) != HAL_OK) {
    timer_handle.Instance = nullptr;
    return false;
  }

  auto master_config = trigger.master_config;
  if (HAL_TIMEx_MasterConfigSynchronization(&timer_handle, &master_config) != HAL_OK) {
    (void)HAL_TIM_Base_DeInit(&timer_handle);
    timer_handle.Instance = nullptr;
    return false;
  }

  __HAL_TIM_SET_COUNTER(&timer_handle, 0U);
  return true;
}

bool start_trigger_timer(dma_adc_backend& backend) noexcept {
  auto& timer_handle = backend.trigger_timer_handle;
  if (timer_handle.Instance == nullptr) {
    return true;
  }

  return HAL_TIM_Base_Start(&timer_handle) == HAL_OK;
}

bool start_dma_backend(const AdcId id) noexcept {
  if constexpr (stm32h5xx::cfg::adc::COUNT == 0U) {
    (void)id;
    return false;
  }

  const auto* const input_config = dma_adc_config(id);
  if (input_config == nullptr) {
    return false;
  }
  const auto* const dma = input_config->config.dma();
  if (dma == nullptr) {
    return false;
  }

  auto* const p_backend = dma_backend_for_instance(input_config->config.instance());
  if (p_backend == nullptr) {
    return false;
  }

  auto& backend = *p_backend;
  if (backend.init_attempted) {
    return backend.running;
  }

  backend.init_attempted = true;

  __HAL_RCC_ADC_CLK_ENABLE();
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  if (!prepare_adc_channels(backend, input_config->config.instance(), dma->frame_count)) {
    return false;
  }

  if (const auto* const trigger = dma->timer_trigger(); trigger != nullptr) {
    if (!init_trigger_timer(backend, *trigger)) {
      return false;
    }
  }

  DMA_NodeConfTypeDef node_config{};
  node_config.NodeType = DMA_GPDMA_LINEAR_NODE;
  node_config.Init.Request = dma->request;
  node_config.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
  node_config.Init.Direction = DMA_PERIPH_TO_MEMORY;
  node_config.Init.SrcInc = DMA_SINC_FIXED;
  node_config.Init.DestInc = DMA_DINC_INCREMENTED;
  node_config.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_HALFWORD;
  node_config.Init.DestDataWidth = DMA_DEST_DATAWIDTH_HALFWORD;
  node_config.Init.SrcBurstLength = 1U;
  node_config.Init.DestBurstLength = 1U;
  node_config.Init.TransferAllocatedPort =
      DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT0;
  node_config.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  node_config.Init.Mode = DMA_NORMAL;
  node_config.TriggerConfig.TriggerPolarity = DMA_TRIG_POLARITY_MASKED;
  node_config.DataHandlingConfig.DataExchange = DMA_EXCHANGE_NONE;
  node_config.DataHandlingConfig.DataAlignment = DMA_DATA_RIGHTALIGN_ZEROPADDED;

  if (HAL_DMAEx_List_BuildNode(&node_config, &backend.dma_node) != HAL_OK ||
      HAL_DMAEx_List_InsertNode(&backend.dma_queue, nullptr, &backend.dma_node) != HAL_OK ||
      HAL_DMAEx_List_SetCircularMode(&backend.dma_queue) != HAL_OK) {
    return false;
  }

  backend.dma_channel_handle.Instance = dma->dma_channel();
  backend.dma_channel_handle.InitLinkedList.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
  backend.dma_channel_handle.InitLinkedList.LinkStepMode = DMA_LSM_FULL_EXECUTION;
  backend.dma_channel_handle.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT0;
  backend.dma_channel_handle.InitLinkedList.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  backend.dma_channel_handle.InitLinkedList.LinkedListMode = DMA_LINKEDLIST_CIRCULAR;

  if (HAL_DMAEx_List_Init(&backend.dma_channel_handle) != HAL_OK ||
      HAL_DMAEx_List_LinkQ(&backend.dma_channel_handle, &backend.dma_queue) != HAL_OK ||
      HAL_DMA_ConfigChannelAttributes(&backend.dma_channel_handle, DMA_CHANNEL_NPRIV) !=
          HAL_OK) {
    return false;
  }

  HAL_NVIC_SetPriority(dma->irq, 6U, 0U);
  HAL_NVIC_EnableIRQ(dma->irq);

  backend.adc_handle.Instance = input_config->config.instance();
  backend.adc_handle.Init = dma->init;
  backend.adc_handle.Init.ScanConvMode =
      backend.channel_count > 1U ? ADC_SCAN_ENABLE : ADC_SCAN_DISABLE;
  backend.adc_handle.Init.EOCSelection =
      backend.channel_count > 1U ? ADC_EOC_SEQ_CONV : ADC_EOC_SINGLE_CONV;
  backend.adc_handle.Init.NbrOfConversion =
      static_cast<uint32_t>(backend.channel_count);
  __HAL_LINKDMA(&backend.adc_handle, DMA_Handle, backend.dma_channel_handle);

  if (HAL_ADC_Init(&backend.adc_handle) != HAL_OK) {
    return false;
  }

  if (!configure_adc_channels(backend, input_config->config.instance())) {
    return false;
  }

  backend.dma_buffer.fill(0U);
  backend.sums.fill(0U);
  backend.sample_counts.fill(0U);
  backend.processed_samples_in_cycle = 0U;

  if (HAL_ADCEx_Calibration_Start(&backend.adc_handle, ADC_SINGLE_ENDED) != HAL_OK ||
      HAL_ADC_Start_DMA(&backend.adc_handle,
                        reinterpret_cast<uint32_t*>(backend.dma_buffer.data()),
                        static_cast<uint32_t>(backend.active_buffer_length)) != HAL_OK) {
    return false;
  }

  if (!start_trigger_timer(backend)) {
    return false;
  }

  backend.running = true;
  return true;
}

uint32_t lock_irq() noexcept {
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

void unlock_irq(const uint32_t primask) noexcept {
  __set_PRIMASK(primask);
}

expected::expected<uint16_t, result> read_dma_average(const AdcId id) noexcept {
  auto* const p_backend = running_dma_backend_for(id);
  if (p_backend == nullptr) {
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }

  auto& backend = *p_backend;
  const auto channel = adc_index(id);
  if (channel >= stm32h5xx::cfg::adc::COUNT) {
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }

  const uint32_t primask = lock_irq();
  const uint64_t sum = backend.sums[channel];
  const uint32_t count = backend.sample_counts[channel];
  backend.sums[channel] = 0U;
  backend.sample_counts[channel] = 0U;
  unlock_irq(primask);

  if (count == 0U) {
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }

  return static_cast<uint16_t>(sum / count);
}

expected::expected<std::optional<uint16_t>, result> try_read_dma_average(
    const AdcId id) noexcept {
  auto* const p_backend = running_dma_backend_for(id);
  if (p_backend == nullptr) {
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }

  auto& backend = *p_backend;
  const auto channel = adc_index(id);
  if (channel >= stm32h5xx::cfg::adc::COUNT) {
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }

  const uint32_t primask = lock_irq();
  const uint64_t sum = backend.sums[channel];
  const uint32_t count = backend.sample_counts[channel];
  backend.sums[channel] = 0U;
  backend.sample_counts[channel] = 0U;
  unlock_irq(primask);

  if (count == 0U) {
    return std::optional<uint16_t>{};
  }

  return std::optional<uint16_t>{static_cast<uint16_t>(sum / count)};
}
}  // namespace

result Adc::start() noexcept {
  opaque_adc::start();
  return result::OK;
}

Adc::Adc(const AdcId id) noexcept : m_id(id), m_opaque(make_opaque(id)) {
}

result Adc::init() noexcept {
  const auto* const config = stm32h5xx::cfg::adc::config_for(m_id);
  if (config == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  if (config->uses_dma()) {
    return start_dma_backend(m_id) ? result::OK : result::RECOVERABLE_ERROR;
  }

  auto& handle = adc_handle(m_id);
  if (handle.Instance != nullptr) {
    return result::OK;
  }

  return m_opaque.init(*config);
}

result Adc::stop() noexcept {
  if (running_dma_backend_for(m_id) != nullptr) {
    return result::OK;
  }

  return m_opaque.stop();
}

expected::expected<uint16_t, result> Adc::read() noexcept {
  if (running_dma_backend_for(m_id) != nullptr) {
    return read_dma_average(m_id);
  }

  uint16_t value{0U};
  const auto status = m_opaque.read(value);
  if (status != result::OK) {
    return expected::unexpected(status);
  }

  return value;
}

expected::expected<std::optional<uint16_t>, result> Adc::try_read() noexcept {
  if (running_dma_backend_for(m_id) != nullptr) {
    return try_read_dma_average(m_id);
  }

  bool has_value{false};
  uint16_t value{0U};
  const auto status = m_opaque.try_read(has_value, value);
  if (status != result::OK) {
    return expected::unexpected(status);
  }

  if (!has_value) {
    return std::optional<uint16_t>{};
  }

  return std::optional<uint16_t>{value};
}

void adc_dma_half_transfer_callback(ADC_HandleTypeDef* hadc) noexcept {
  auto* const p_backend = dma_backend_from_handle(hadc);
  if (p_backend == nullptr) {
    return;
  }

  auto& backend = *p_backend;
  const auto half_buffer_length = backend.active_buffer_length / 2U;
  accumulate_dma_until(backend, half_buffer_length);
}

void adc_dma_full_transfer_callback(ADC_HandleTypeDef* hadc) noexcept {
  auto* const p_backend = dma_backend_from_handle(hadc);
  if (p_backend == nullptr) {
    return;
  }

  auto& backend = *p_backend;
  accumulate_dma_until(backend, backend.active_buffer_length);
  backend.processed_samples_in_cycle = 0U;
}

void adc_dma_irq_handler(ADC_TypeDef* const p_instance) noexcept {
  auto* const p_backend = dma_backend_for_instance(p_instance);
  if (p_backend == nullptr) {
    return;
  }

  HAL_DMA_IRQHandler(&p_backend->dma_channel_handle);
}
}  // namespace ru::driver

extern "C" void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
  ru::driver::adc_dma_half_transfer_callback(hadc);
}

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  ru::driver::adc_dma_full_transfer_callback(hadc);
}

extern "C" void GPDMA1_Channel0_IRQHandler(void) {
  ru::driver::adc_dma_irq_handler(ADC1);
}

extern "C" void GPDMA1_Channel1_IRQHandler(void) {
  ru::driver::adc_dma_irq_handler(ADC2);
}
