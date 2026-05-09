#include <algorithm>
#include <array>

#include "adc.hpp"
#include "mapping.hpp"
#include "utils/common.hpp"

using namespace ru::driver;

namespace ru::driver {
namespace {
constexpr std::size_t adc_index(const AdcId id) noexcept {
  return static_cast<std::size_t>(id);
}

constexpr std::size_t k_adc_count = static_cast<std::size_t>(AdcId::COUNT);
constexpr std::size_t k_dma_frames = 256U;
constexpr std::size_t k_dma_buffer_capacity =
    (k_adc_count == 0U ? 1U : k_adc_count) * k_dma_frames;
constexpr std::size_t k_dma_backend_count = 2U;

struct adc_dma_channel_config {
  AdcId id;
  ADC_TypeDef* p_instance;
  GPIO_TypeDef* p_port;
  uint16_t pin;
  uint32_t channel;
};

struct adc_dma_instance_config {
  ADC_TypeDef* p_instance;
  uint32_t request;
  DMA_Channel_TypeDef* p_dma_channel;
  IRQn_Type irq;
};

ADC_HandleTypeDef& adc_handle(const AdcId id) noexcept {
  static std::array<ADC_HandleTypeDef, k_adc_count> handles{};
  static ADC_HandleTypeDef dummy_handle{};
  if constexpr (k_adc_count == 0U) {
    (void)id;
    return dummy_handle;
  }

  const auto index = adc_index(id);
  return index < handles.size() ? handles[index] : handles.front();
}

constexpr std::array<adc_dma_channel_config, k_adc_count> k_adc_dma_configs = {
#define RU_STM32H5XX_ADC_DMA_ENTRY(name, instance, port, pin, channel) \
  adc_dma_channel_config{AdcId::name, instance, port, pin, channel},
    RU_STM32H5XX_ADC_MAP(RU_STM32H5XX_ADC_DMA_ENTRY)
#undef RU_STM32H5XX_ADC_DMA_ENTRY
};

constexpr std::array<adc_dma_instance_config, k_dma_backend_count> k_dma_instance_configs{{
    {ADC1, GPDMA1_REQUEST_ADC1, GPDMA1_Channel0, GPDMA1_Channel0_IRQn},
    {ADC2, GPDMA1_REQUEST_ADC2, GPDMA1_Channel1, GPDMA1_Channel1_IRQn},
}};

constexpr opaque_adc make_opaque(const AdcId id) noexcept {
  switch (id) {
#define RU_STM32H5XX_ADC_CASE(name, instance, port, pin, channel)                        \
    case AdcId::name:                                                                    \
      return opaque_adc{instance, port, pin, channel};
    RU_STM32H5XX_ADC_MAP(RU_STM32H5XX_ADC_CASE)
#undef RU_STM32H5XX_ADC_CASE
    default:
      return opaque_adc{};
  }
}

struct dma_adc_backend {
  ADC_HandleTypeDef adc_handle{};
  DMA_NodeTypeDef dma_node{};
  DMA_QListTypeDef dma_queue{};
  DMA_HandleTypeDef dma_channel_handle{};
  std::array<uint16_t, k_dma_buffer_capacity> dma_buffer{};
  std::array<AdcId, k_adc_count> ids{};
  std::array<uint64_t, k_adc_count> sums{};
  std::array<uint32_t, k_adc_count> sample_counts{};
  std::size_t channel_count{0U};
  std::size_t active_buffer_length{0U};
  std::size_t processed_samples_in_cycle{0U};
  bool init_attempted{false};
  bool running{false};
};

std::array<dma_adc_backend, k_dma_backend_count>& dma_backends() noexcept {
  static std::array<dma_adc_backend, k_dma_backend_count> backends{};
  return backends;
}

const adc_dma_channel_config* adc_config(const AdcId id) noexcept {
  for (const auto& config : k_adc_dma_configs) {
    if (config.id == id) {
      return &config;
    }
  }

  return nullptr;
}

const adc_dma_instance_config* dma_instance_config(
    ADC_TypeDef* const p_instance) noexcept {
  for (const auto& config : k_dma_instance_configs) {
    if (config.p_instance == p_instance) {
      return &config;
    }
  }

  return nullptr;
}

dma_adc_backend* dma_backend_for_instance(ADC_TypeDef* const p_instance) noexcept {
  auto& backends = dma_backends();
  for (std::size_t index = 0U; index < k_dma_instance_configs.size(); ++index) {
    if (k_dma_instance_configs[index].p_instance == p_instance) {
      return &backends[index];
    }
  }

  return nullptr;
}

dma_adc_backend* running_dma_backend_for(const AdcId id) noexcept {
  const auto* const config = adc_config(id);
  if (config == nullptr) {
    return nullptr;
  }

  auto* const backend = dma_backend_for_instance(config->p_instance);
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
    if (channel < k_adc_count) {
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
                          ADC_TypeDef* const p_instance) noexcept {
  std::size_t rank = 0U;
  for (const auto& config : k_adc_dma_configs) {
    if (config.p_instance != p_instance) {
      continue;
    }

    if (rank >= backend.ids.size()) {
      return false;
    }

    init_pin(config.p_port, config.pin, GPIO_MODE_ANALOG, GPIO_NOPULL);
    backend.ids[rank] = config.id;
    ++rank;
  }

  backend.channel_count = rank;
  backend.active_buffer_length = backend.channel_count * k_dma_frames;
  return backend.channel_count != 0U &&
         backend.active_buffer_length <= backend.dma_buffer.size();
}

bool configure_adc_channels(dma_adc_backend& backend,
                            ADC_TypeDef* const p_instance) noexcept {
  ADC_ChannelConfTypeDef channel_config{};
  channel_config.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
  channel_config.SingleDiff = ADC_SINGLE_ENDED;
  channel_config.OffsetNumber = ADC_OFFSET_NONE;
  channel_config.Offset = 0U;
  channel_config.OffsetSign = ADC_OFFSET_SIGN_NEGATIVE;
  channel_config.OffsetSaturation = DISABLE;

  std::size_t rank = 0U;
  for (const auto& config : k_adc_dma_configs) {
    if (config.p_instance != p_instance) {
      continue;
    }

    if (rank >= backend.channel_count) {
      return false;
    }

    channel_config.Channel = config.channel;
    channel_config.Rank = static_cast<uint32_t>(rank + 1U);

    if (HAL_ADC_ConfigChannel(&backend.adc_handle, &channel_config) != HAL_OK) {
      return false;
    }

    ++rank;
  }

  return rank == backend.channel_count;
}

bool start_dma_backend(const AdcId id) noexcept {
  if constexpr (k_adc_count == 0U) {
    (void)id;
    return false;
  }

  const auto* const input_config = adc_config(id);
  if (input_config == nullptr) {
    return false;
  }

  const auto* const instance_config = dma_instance_config(input_config->p_instance);
  auto* const p_backend = dma_backend_for_instance(input_config->p_instance);
  if (instance_config == nullptr || p_backend == nullptr) {
    return false;
  }

  auto& backend = *p_backend;
  if (backend.init_attempted) {
    return backend.running;
  }

  backend.init_attempted = true;

  __HAL_RCC_ADC_CLK_ENABLE();
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  if (!prepare_adc_channels(backend, input_config->p_instance)) {
    return false;
  }

  DMA_NodeConfTypeDef node_config{};
  node_config.NodeType = DMA_GPDMA_LINEAR_NODE;
  node_config.Init.Request = instance_config->request;
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

  backend.dma_channel_handle.Instance = instance_config->p_dma_channel;
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

  HAL_NVIC_SetPriority(instance_config->irq, 6U, 0U);
  HAL_NVIC_EnableIRQ(instance_config->irq);

  backend.adc_handle.Instance = input_config->p_instance;
  backend.adc_handle.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  backend.adc_handle.Init.Resolution = ADC_RESOLUTION_12B;
  backend.adc_handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  backend.adc_handle.Init.ScanConvMode =
      backend.channel_count > 1U ? ADC_SCAN_ENABLE : ADC_SCAN_DISABLE;
  backend.adc_handle.Init.EOCSelection =
      backend.channel_count > 1U ? ADC_EOC_SEQ_CONV : ADC_EOC_SINGLE_CONV;
  backend.adc_handle.Init.LowPowerAutoWait = DISABLE;
  backend.adc_handle.Init.ContinuousConvMode = ENABLE;
  backend.adc_handle.Init.NbrOfConversion =
      static_cast<uint32_t>(backend.channel_count);
  backend.adc_handle.Init.DiscontinuousConvMode = DISABLE;
  backend.adc_handle.Init.NbrOfDiscConversion = 1U;
  backend.adc_handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  backend.adc_handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  backend.adc_handle.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
  backend.adc_handle.Init.DMAContinuousRequests = ENABLE;
  backend.adc_handle.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  backend.adc_handle.Init.OversamplingMode = DISABLE;
  __HAL_LINKDMA(&backend.adc_handle, DMA_Handle, backend.dma_channel_handle);

  if (HAL_ADC_Init(&backend.adc_handle) != HAL_OK) {
    return false;
  }

  if (!configure_adc_channels(backend, input_config->p_instance)) {
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
  if (channel >= k_adc_count) {
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
  if (channel >= k_adc_count) {
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
  return result::OK;
}

Adc::Adc(const AdcId id) noexcept : m_id(id), m_opaque(make_opaque(id)) {
}

result Adc::init() noexcept {
  if (start_dma_backend(m_id)) {
    return result::OK;
  }

  return m_opaque.init(&adc_handle(m_id));
}

result Adc::stop() noexcept {
  if (running_dma_backend_for(m_id) != nullptr) {
    return result::OK;
  }

  return m_opaque.stop(&adc_handle(m_id));
}

expected::expected<uint16_t, result> Adc::read() noexcept {
  if (running_dma_backend_for(m_id) != nullptr) {
    return read_dma_average(m_id);
  }

  uint16_t value{0U};
  const auto status = m_opaque.read(&adc_handle(m_id), value);
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
  const auto status = m_opaque.try_read(&adc_handle(m_id), has_value, value);
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
