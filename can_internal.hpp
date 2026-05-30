#pragma once

#include <array>
#include <optional>

#include "can/bx_can.hpp"
#include "can/m_can.hpp"
#include "mapping.hpp"
#include "opaque_can.hpp"
#include "stm_common.hpp"

namespace ru::driver::can_internal {
inline constexpr std::size_t k_std_filter_slots = 28U;
inline constexpr std::size_t k_ext_filter_slots = 8U;

constexpr uint32_t fdcan_dlc_from_length(const uint8_t len) noexcept {
  switch (len) {
    case 0U:
      return FDCAN_DLC_BYTES_0;
    case 1U:
      return FDCAN_DLC_BYTES_1;
    case 2U:
      return FDCAN_DLC_BYTES_2;
    case 3U:
      return FDCAN_DLC_BYTES_3;
    case 4U:
      return FDCAN_DLC_BYTES_4;
    case 5U:
      return FDCAN_DLC_BYTES_5;
    case 6U:
      return FDCAN_DLC_BYTES_6;
    case 7U:
      return FDCAN_DLC_BYTES_7;
    case 8U:
      return FDCAN_DLC_BYTES_8;
    case 12U:
      return FDCAN_DLC_BYTES_12;
    case 16U:
      return FDCAN_DLC_BYTES_16;
    case 20U:
      return FDCAN_DLC_BYTES_20;
    case 24U:
      return FDCAN_DLC_BYTES_24;
    case 32U:
      return FDCAN_DLC_BYTES_32;
    case 48U:
      return FDCAN_DLC_BYTES_48;
    default:
      return FDCAN_DLC_BYTES_64;
  }
}

constexpr uint8_t fdcan_length_from_dlc(const uint32_t dlc) noexcept {
  switch (dlc) {
    case FDCAN_DLC_BYTES_0:
      return 0U;
    case FDCAN_DLC_BYTES_1:
      return 1U;
    case FDCAN_DLC_BYTES_2:
      return 2U;
    case FDCAN_DLC_BYTES_3:
      return 3U;
    case FDCAN_DLC_BYTES_4:
      return 4U;
    case FDCAN_DLC_BYTES_5:
      return 5U;
    case FDCAN_DLC_BYTES_6:
      return 6U;
    case FDCAN_DLC_BYTES_7:
      return 7U;
    case FDCAN_DLC_BYTES_8:
      return 8U;
    case FDCAN_DLC_BYTES_12:
      return 12U;
    case FDCAN_DLC_BYTES_16:
      return 16U;
    case FDCAN_DLC_BYTES_20:
      return 20U;
    case FDCAN_DLC_BYTES_24:
      return 24U;
    case FDCAN_DLC_BYTES_32:
      return 32U;
    case FDCAN_DLC_BYTES_48:
      return 48U;
    default:
      return 64U;
  }
}

FDCAN_HandleTypeDef& handle(const opaque_can& config) noexcept;
bool initialized(const opaque_can& config) noexcept;
bool started(const opaque_can& config) noexcept;
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

inline const stm32h5xx::cfg::can_config* config_for(const M_canId id) noexcept {
  switch (id) {
#define RU_STM32H5XX_M_CAN_CONFIG(name, config) \
    case M_canId::name:                         \
      return &config;
    RU_STM32H5XX_M_CAN_MAP(RU_STM32H5XX_M_CAN_CONFIG)
#undef RU_STM32H5XX_M_CAN_CONFIG
    default:
      return nullptr;
  }
}

inline const stm32h5xx::cfg::can_config* config_for(const Bx_canId id) noexcept {
  switch (id) {
#define RU_STM32H5XX_BX_CAN_CONFIG(name, config) \
    case Bx_canId::name:                         \
      return &config;
    RU_STM32H5XX_BX_CAN_MAP(RU_STM32H5XX_BX_CAN_CONFIG)
#undef RU_STM32H5XX_BX_CAN_CONFIG
    default:
      return nullptr;
  }
}

inline opaque_can make_opaque(FDCAN_GlobalTypeDef* const p_instance) noexcept {
  if (p_instance == FDCAN1) {
    return opaque_can{can_controller_key::Fdcan1};
  }
#if defined(FDCAN2)
  if (p_instance == FDCAN2) {
    return opaque_can{can_controller_key::Fdcan2};
  }
#else
  (void)p_instance;
#endif

  return opaque_can{};
}

inline opaque_can make_opaque(const M_canId id) noexcept {
  const auto* const config = config_for(id);
  if (config == nullptr) {
    return opaque_can{};
  }

  return make_opaque(config->instance());
}

inline opaque_can make_opaque(const Bx_canId id) noexcept {
  const auto* const config = config_for(id);
  if (config == nullptr) {
    return opaque_can{};
  }

  return make_opaque(config->instance());
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

  const auto status = fn();
  if (status != result::OK) {
    if (restart && HAL_FDCAN_Start(&hw_handle) == HAL_OK) {
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

  return refresh_notifications(config);
}

result init_controller(const opaque_can& config,
                       const stm32h5xx::cfg::can_config& init_config) noexcept;
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
