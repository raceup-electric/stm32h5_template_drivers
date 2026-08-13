#include "spi.hpp"

#include <algorithm>
#include <limits>

#include "mapping.hpp"
#include "stm_common.hpp"
#include "stm32h5xx_hal.h"

// Assuming FreeRTOS context based on previous discussions
#include "FreeRTOS.h"
#include "task.h"

namespace ru::driver {

namespace {
// Since we only have one instance, we just need single static variables.
static SPI_HandleTypeDef g_spi_handle{};
static DMA_HandleTypeDef g_dma_tx_handle{};
static DMA_HandleTypeDef g_dma_rx_handle{};

// --- Legacy blocking transfer() task-wake mechanism ---
static volatile TaskHandle_t g_spi_task_handle = nullptr;

// --- Streaming (ping-pong) mechanism ---
// Callback invoked from ISR right after each streamed transfer
// completes/errors; expected to call arm() with the next buffer pair.
static volatile TransferEventCallback g_stream_cb = nullptr;
static void* g_stream_cb_ctx = nullptr;
// Task notified (non-blocking wake, for bookkeeping only) after each
// streamed transfer. Independent from g_spi_task_handle above.
static volatile TaskHandle_t g_stream_task_handle = nullptr;

const stm32h5xx::cfg::spi_config* config_for(const SpiId id) noexcept {
  switch (id) {
#define RU_STM32H5XX_SPI_CONFIG(name, config) \
    case SpiId::name:                         \
      return &config;
    RU_STM32H5XX_SPI_MAP(RU_STM32H5XX_SPI_CONFIG)
#undef RU_STM32H5XX_SPI_CONFIG
    default:
      return nullptr;
  }
}

// Extracted from the old transfer() step 1. Shared by the blocking API
// and the streaming error-callback path.
// NOTE: this can be called from ISR context (streaming error path).
// HAL_SPI_Abort() is expected to return promptly here because it is only
// ever invoked immediately after the peripheral has already signalled an
// error (i.e. it is not actively shifting), but this is worth confirming
// with a logic analyzer / IRQ latency trace if OVR errors are frequent.
void recover_spi_error() noexcept {
  // 1. Aggressive SPI Slave Error Recovery & State Reset
  if (g_spi_handle.State != HAL_SPI_STATE_READY || 
      g_spi_handle.ErrorCode != HAL_SPI_ERROR_NONE || 
      __HAL_SPI_GET_FLAG(&g_spi_handle, SPI_FLAG_OVR)) {
    
    // HAL_SPI_Abort is crucial here: it forces g_spi_handle.State back to HAL_SPI_STATE_READY
    HAL_SPI_Abort(&g_spi_handle);
    
    __HAL_SPI_CLEAR_OVRFLAG(&g_spi_handle);
    __HAL_SPI_CLEAR_MODFFLAG(&g_spi_handle);
    
    // If soft-clear fails and it's still locked up, force a hardware reset
    if (g_spi_handle.State != HAL_SPI_STATE_READY) {
       __HAL_RCC_SPI1_FORCE_RESET();
       __HAL_RCC_SPI1_RELEASE_RESET();
       HAL_SPI_Init(&g_spi_handle); 
    }
  }
}
}  // namespace

result Spi::start() noexcept {
  return result::OK;
}

result Spi::init() noexcept {
  const auto* const p_config = config_for(m_id);
  if (p_config == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  g_spi_handle.Instance = p_config->instance();
  g_spi_handle.Init = p_config->init;

  // =========================================================================
  // INTERNAL HARDWARE MANAGEMENT (STM32H5)
  // =========================================================================
  if (g_spi_handle.Instance == SPI1) {
    // 1. Enable Clocks
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPDMA1_CLK_ENABLE(); // H5 uses GPDMA

    // 2. Configure GPIO (SPI Slave requires NSS)
    GPIO_InitTypeDef init{};
    init.Mode = GPIO_MODE_AF_PP;
    init.Pull = GPIO_PULLDOWN;
    init.Speed = GPIO_SPEED_FREQ_HIGH;
    init.Alternate = GPIO_AF5_SPI1;

    init.Pin = GPIO_PIN_4; // NSS
    init.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &init);
    init.Pull = GPIO_PULLDOWN;
    init.Pin = GPIO_PIN_5; // SCK
    HAL_GPIO_Init(GPIOA, &init);
    init.Pin = GPIO_PIN_6; // MISO
    HAL_GPIO_Init(GPIOA, &init);
    init.Pin = GPIO_PIN_7; // MOSI
    HAL_GPIO_Init(GPIOA, &init);

    // 3. Configure GPDMA RX
    g_dma_rx_handle.Instance                 = GPDMA1_Channel0;
    g_dma_rx_handle.Init.Request             = GPDMA1_REQUEST_SPI1_RX;
    g_dma_rx_handle.Init.BlkHWRequest        = DMA_BREQ_SINGLE_BURST;
    g_dma_rx_handle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    g_dma_rx_handle.Init.SrcInc              = DMA_SINC_FIXED;
    g_dma_rx_handle.Init.DestInc             = DMA_DINC_INCREMENTED;
    g_dma_rx_handle.Init.SrcDataWidth        = DMA_SRC_DATAWIDTH_BYTE;
    g_dma_rx_handle.Init.DestDataWidth       = DMA_DEST_DATAWIDTH_BYTE;
    g_dma_rx_handle.Init.Priority            = DMA_HIGH_PRIORITY;
    g_dma_rx_handle.Init.SrcBurstLength      = 1;
    g_dma_rx_handle.Init.DestBurstLength     = 1;
    g_dma_rx_handle.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT0;
    g_dma_rx_handle.Init.TransferEventMode   = DMA_TCEM_BLOCK_TRANSFER;
    g_dma_rx_handle.Init.Mode                = DMA_NORMAL;
    if (HAL_DMA_Init(&g_dma_rx_handle) != HAL_OK) return result::UNRECOVERABLE_ERROR;
    __HAL_LINKDMA(&g_spi_handle, hdmarx, g_dma_rx_handle);

    // 4. Configure GPDMA TX
    g_dma_tx_handle.Instance                 = GPDMA1_Channel1;
    g_dma_tx_handle.Init.Request             = GPDMA1_REQUEST_SPI1_TX;
    g_dma_tx_handle.Init.BlkHWRequest        = DMA_BREQ_SINGLE_BURST;
    g_dma_tx_handle.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    g_dma_tx_handle.Init.SrcInc              = DMA_SINC_INCREMENTED;
    g_dma_tx_handle.Init.DestInc             = DMA_DINC_FIXED;
    g_dma_tx_handle.Init.SrcDataWidth        = DMA_SRC_DATAWIDTH_BYTE;
    g_dma_tx_handle.Init.DestDataWidth       = DMA_DEST_DATAWIDTH_BYTE;
    g_dma_tx_handle.Init.Priority            = DMA_LOW_PRIORITY_LOW_WEIGHT;
    g_dma_tx_handle.Init.SrcBurstLength      = 1;
    g_dma_tx_handle.Init.DestBurstLength     = 1;
    g_dma_tx_handle.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT0;
    g_dma_tx_handle.Init.TransferEventMode   = DMA_TCEM_BLOCK_TRANSFER;
    g_dma_tx_handle.Init.Mode                = DMA_NORMAL;
    if (HAL_DMA_Init(&g_dma_tx_handle) != HAL_OK) return result::UNRECOVERABLE_ERROR;
    __HAL_LINKDMA(&g_spi_handle, hdmatx, g_dma_tx_handle);

    // 5. Configure NVIC
    HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);
    HAL_NVIC_SetPriority(GPDMA1_Channel1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel1_IRQn);
    HAL_NVIC_SetPriority(SPI1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
  }

  // 6. Initialize SPI Logic State Machine
  return from_hal_status(HAL_SPI_Init(&g_spi_handle));
}

result Spi::stop() noexcept {
  if (g_spi_handle.Instance == SPI1) {
    HAL_NVIC_DisableIRQ(SPI1_IRQn);
    HAL_NVIC_DisableIRQ(GPDMA1_Channel0_IRQn);
    HAL_NVIC_DisableIRQ(GPDMA1_Channel1_IRQn);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

    __HAL_RCC_SPI1_CLK_DISABLE();
  }

  if (g_spi_handle.hdmarx != nullptr) HAL_DMA_DeInit(g_spi_handle.hdmarx);
  if (g_spi_handle.hdmatx != nullptr) HAL_DMA_DeInit(g_spi_handle.hdmatx);

  // Streaming state is no longer valid once the peripheral is torn down.
  g_stream_cb = nullptr;
  g_stream_cb_ctx = nullptr;
  g_stream_task_handle = nullptr;

  return from_hal_status(HAL_SPI_DeInit(&g_spi_handle));
}

result Spi::transfer(const uint8_t* tx_data, uint8_t* rx_data, const size_t len,
                     const Timestamp timeout_uS) noexcept {
  if (len == 0U || tx_data == nullptr || rx_data == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  // 1. Aggressive SPI Slave Error Recovery
  recover_spi_error();

  // 2. STM32H5 Cache Coherency (Cortex-M33)
  // NOTE: tx_data and rx_data MUST be allocated with 32-byte alignment in memory: alignas(32)
  const int32_t cache_len = (static_cast<int32_t>(len) + 31) & ~31;
  if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
      SCB_CleanDCache_by_Addr(reinterpret_cast<uint32_t*>(const_cast<uint8_t*>(tx_data)), cache_len);
  }
  // 3. Register current task to be awoken by the ISR
  g_spi_task_handle = xTaskGetCurrentTaskHandle();

  // Clear any pending notifications before starting
  ulTaskNotifyTake(pdTRUE, 0);

  // Before HAL_SPI_TransmitReceive_DMA:
  if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
    SCB_CleanInvalidateDCache_by_Addr(reinterpret_cast<uint32_t*>(rx_data), cache_len);
  }
  // 4. Start Non-Blocking DMA Transfer
  auto status = HAL_SPI_TransmitReceive_DMA(&g_spi_handle,
                                            const_cast<uint8_t*>(tx_data),
                                            rx_data,
                                            static_cast<uint16_t>(len));
  if (status != HAL_OK) {
    g_spi_task_handle = nullptr;
    return from_hal_status(status);
  }

  // 5. Block Task natively (No polling!) waiting for ISR notification
  const uint32_t timeout_ms = std::max<uint32_t>(us_to_ms(timeout_uS), 1U);

  BaseType_t wait_res = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout_ms));

  if (wait_res != pdTRUE) {
    // Timeout hit before ISR fired
    HAL_SPI_Abort(&g_spi_handle);
    return result::RECOVERABLE_ERROR;
  }

  if (g_spi_task_handle != nullptr) {
    g_spi_task_handle = nullptr; // Safety clear
  }

  // 6. Post-Transfer Cache Invalidation
  if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
    SCB_InvalidateDCache_by_Addr(reinterpret_cast<uint32_t*>(rx_data), cache_len);
  }

  if (g_spi_handle.ErrorCode != HAL_SPI_ERROR_NONE) {
    return result::UNRECOVERABLE_ERROR;
  }

  return result::OK;
}

// --- Streaming API -------------------------------------------------------

void Spi::set_stream_callback(TransferEventCallback cb, void* ctx) noexcept {
  g_stream_cb_ctx = ctx;
  g_stream_cb = cb; // publish ctx before cb so the ISR never sees cb!=null with a stale ctx
}

void Spi::set_stream_task_handle(TaskHandle_t handle) noexcept {
  g_stream_task_handle = handle;
}

result Spi::arm(const uint8_t* tx_data, uint8_t* rx_data, const size_t len) noexcept {
  if (len == 0U || tx_data == nullptr || rx_data == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  const int32_t cache_len = (static_cast<int32_t>(len) + 31) & ~31;
  if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
      SCB_CleanDCache_by_Addr(reinterpret_cast<uint32_t*>(const_cast<uint8_t*>(tx_data)), cache_len);
      SCB_CleanInvalidateDCache_by_Addr(reinterpret_cast<uint32_t*>(rx_data), cache_len);
  }

  auto status = HAL_SPI_TransmitReceive_DMA(&g_spi_handle,
                                            const_cast<uint8_t*>(tx_data),
                                            rx_data,
                                            static_cast<uint16_t>(len));
  return from_hal_status(status);
}

bool Spi::spiReadyToArm() noexcept {
  return g_spi_handle.State == HAL_SPI_STATE_READY;
}

}  // namespace ru::driver

// =========================================================================
// INTERRUPT SERVICE ROUTINES & CALLBACKS
// =========================================================================
extern "C" {

void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi) {
  (void)hspi;
}
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi) {
  (void)hspi;
}

void SPI1_IRQHandler(void) {
  HAL_SPI_IRQHandler(&ru::driver::g_spi_handle);
}

void GPDMA1_Channel0_IRQHandler(void) {
  HAL_DMA_IRQHandler(&ru::driver::g_dma_rx_handle);
}

void GPDMA1_Channel1_IRQHandler(void) {
  HAL_DMA_IRQHandler(&ru::driver::g_dma_tx_handle);
}

// -------------------------------------------------------------------------
// HAL Callbacks: These are called automatically by HAL_SPI_IRQHandler
// or HAL_DMA_IRQHandler when the transfer finishes or errors out.
// -------------------------------------------------------------------------

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance != SPI1) return;

  // Streaming path: re-arm FIRST (this is the whole point — minimize the
  // gap between "transfer done" and "next transfer armed"), then notify.
  if (ru::driver::g_stream_cb != nullptr) {
    ru::driver::g_stream_cb(ru::driver::g_stream_cb_ctx, /*error=*/false);
  }
  if (ru::driver::g_stream_task_handle != nullptr) {
    BaseType_t stream_woken = pdFALSE;
    vTaskNotifyGiveFromISR(ru::driver::g_stream_task_handle, &stream_woken);
    portYIELD_FROM_ISR(stream_woken);
  }

  // Legacy blocking-transfer() path
  if (ru::driver::g_spi_task_handle != nullptr) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(ru::driver::g_spi_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance != SPI1) return;

  if (ru::driver::g_stream_cb != nullptr) {
    // Recover the peripheral before re-arming.
    ru::driver::recover_spi_error();
    ru::driver::g_stream_cb(ru::driver::g_stream_cb_ctx, /*error=*/true);
  }
  if (ru::driver::g_stream_task_handle != nullptr) {
    BaseType_t stream_woken = pdFALSE;
    vTaskNotifyGiveFromISR(ru::driver::g_stream_task_handle, &stream_woken);
    portYIELD_FROM_ISR(stream_woken);
  }

  if (ru::driver::g_spi_task_handle != nullptr) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(ru::driver::g_spi_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

} // extern "C"