#pragma once

#include <array>
#include <optional>

#include "can/bx_can.hpp"
#include "can/m_can.hpp"
#include "mapping.hpp"
#include "opaque_can.hpp"
#include "utils/common.hpp"

namespace ru::driver::can_internal {
inline constexpr uint8_t k_default_irq_priority = 5U;
inline constexpr std::size_t k_std_filter_slots = 28U;
inline constexpr std::size_t k_ext_filter_slots = 8U;
inline constexpr uint32_t k_rx_interrupt_mask_fifo0 = FDCAN_IT_RX_FIFO0_NEW_MESSAGE;
inline constexpr uint32_t k_rx_interrupt_mask_fifo1 = FDCAN_IT_RX_FIFO1_NEW_MESSAGE;
inline constexpr uint32_t k_bus_off_interrupt_mask = FDCAN_IT_BUS_OFF;
inline constexpr uint32_t k_optional_error_interrupt_mask =
    FDCAN_IT_ERROR_WARNING | FDCAN_IT_ERROR_PASSIVE |
    FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR |
    FDCAN_IT_RAM_ACCESS_FAILURE | FDCAN_IT_ERROR_LOGGING_OVERFLOW |
    FDCAN_IT_RESERVED_ADDRESS_ACCESS;
inline constexpr uint32_t k_error_interrupt_mask =
    k_bus_off_interrupt_mask | k_optional_error_interrupt_mask;
inline constexpr uint32_t k_tx_interrupt_mask = FDCAN_IT_TX_COMPLETE;
inline constexpr uint32_t k_all_interrupt_mask =
    k_rx_interrupt_mask_fifo0 | k_rx_interrupt_mask_fifo1 | k_error_interrupt_mask |
    k_tx_interrupt_mask;

FDCAN_HandleTypeDef& handle(const opaque_can& config) noexcept;
bool& initialized(const opaque_can& config) noexcept;
bool& started(const opaque_can& config) noexcept;
std::array<uint8_t, 2>& rx_priorities(const opaque_can& config) noexcept;
std::array<bool, 2>& rx_interrupts(const opaque_can& config) noexcept;
uint8_t& error_priority(const opaque_can& config) noexcept;
bool& error_interrupt(const opaque_can& config) noexcept;
std::optional<M_fifo>& not_matching(const opaque_can& config) noexcept;
std::array<std::optional<M_filter>, k_std_filter_slots>& m_filters(
    const opaque_can& config) noexcept;
std::array<bool, k_std_filter_slots>& m_filter_enabled(const opaque_can& config) noexcept;
std::array<std::optional<Bx_filter>, k_std_filter_slots>& bx_filters(
    const opaque_can& config) noexcept;
std::array<bool, k_std_filter_slots>& bx_filter_enabled(const opaque_can& config) noexcept;
std::array<void (*)(CanMessageTs), 2>& rx_callbacks(const opaque_can& config) noexcept;
void (*&txfull_callback(const opaque_can& config))();

constexpr opaque_can can_opaque_from_instance(
    FDCAN_GlobalTypeDef* const p_instance) noexcept {
  if (p_instance == FDCAN1) {
    return opaque_can{
        FDCAN1, GPIOD, GPIO_PIN_0, GPIOD, GPIO_PIN_1, GPIO_AF9_FDCAN1};
  }
#if defined(FDCAN2) && defined(GPIO_AF9_FDCAN2)
  if (p_instance == FDCAN2) {
    return opaque_can{
        FDCAN2, GPIOB, GPIO_PIN_5, GPIOB, GPIO_PIN_13, GPIO_AF9_FDCAN2};
  }
#endif
  return opaque_can{};
}

constexpr opaque_can make_opaque(const M_canId id) noexcept {
  switch (id) {
#define RU_STM32H5XX_M_CAN_CASE(name, instance) \
    case M_canId::name:                         \
      return can_opaque_from_instance(instance);
    RU_STM32H5XX_M_CAN_MAP(RU_STM32H5XX_M_CAN_CASE)
#undef RU_STM32H5XX_M_CAN_CASE
    default:
      return opaque_can{};
  }
}

constexpr opaque_can make_opaque(const Bx_canId id) noexcept {
  switch (id) {
#define RU_STM32H5XX_BX_CAN_CASE(name, instance) \
    case Bx_canId::name:                         \
      return can_opaque_from_instance(instance);
    RU_STM32H5XX_BX_CAN_MAP(RU_STM32H5XX_BX_CAN_CASE)
#undef RU_STM32H5XX_BX_CAN_CASE
    default:
      return opaque_can{};
  }
}

constexpr opaque_can make_opaque(const Flex_canId id) noexcept {
  (void)id;
  return opaque_can{};
}

constexpr opaque_can make_opaque(const Multi_canId id) noexcept {
  (void)id;
  return opaque_can{};
}

constexpr std::size_t fifo_index(const M_fifo fifo) noexcept {
  return fifo == M_fifo::FIFO1 ? 1U : 0U;
}

constexpr std::size_t fifo_index(const Bx_fifo fifo) noexcept {
  return fifo == Bx_fifo::FIFO1 ? 1U : 0U;
}

constexpr M_fifo to_m_fifo(const Bx_fifo fifo) noexcept {
  return fifo == Bx_fifo::FIFO1 ? M_fifo::FIFO1 : M_fifo::FIFO0;
}

constexpr result unrecoverable_result() noexcept {
  return result::UNRECOVERABLE_ERROR;
}

template <typename T>
expected::expected<T, result> unrecoverable_expected() noexcept {
  return expected::unexpected(result::UNRECOVERABLE_ERROR);
}

result refresh_notifications(const opaque_can& config) noexcept;
result apply_global_filter(const opaque_can& config) noexcept;

template <typename Fn>
result with_stopped_controller(const opaque_can& config, Fn&& fn) noexcept {
  auto& hw_handle = handle(config);
  const bool restart = started(config);
  if (restart && HAL_FDCAN_Stop(&hw_handle) != HAL_OK) {
    return result::RECOVERABLE_ERROR;
  }

  started(config) = false;
  const auto status = fn();
  if (status != result::OK) {
    if (restart && HAL_FDCAN_Start(&hw_handle) == HAL_OK) {
      started(config) = true;
      (void)refresh_notifications(config);
    }
    return status;
  }

  if (!restart) {
    return result::OK;
  }

  if (HAL_FDCAN_Start(&hw_handle) != HAL_OK) {
    return result::RECOVERABLE_ERROR;
  }

  started(config) = true;
  return refresh_notifications(config);
}

result init_controller(const opaque_can& config) noexcept;
result stop_controller(const opaque_can& config) noexcept;
expected::expected<CanMessageTs, result> read_fifo_message(const opaque_can& config,
                                                          M_fifo fifo) noexcept;
result write_controller_message(const opaque_can& config,
                                const CanMessage& message) noexcept;
result configure_m_filter(const opaque_can& config, const M_filter& filter, uint8_t id,
                          bool enabled) noexcept;
result configure_bx_filter(const opaque_can& config, const Bx_filter& filter, uint8_t id,
                           bool enabled) noexcept;
void service_rx_fifo(const opaque_can& config, M_fifo fifo) noexcept;
}  // namespace ru::driver::can_internal
