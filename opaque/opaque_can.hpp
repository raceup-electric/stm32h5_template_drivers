#pragma once

#include <cstdint>

#include "stm32h5xx_hal.h"

namespace ru::driver {


inline void enable_fdcan_clock(FDCAN_GlobalTypeDef* const p_instance) noexcept {
  (void)p_instance;
  __HAL_RCC_FDCAN_CLK_ENABLE();
}

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









enum class can_controller_key : uint8_t { Invalid, Fdcan1, Fdcan2 };

struct opaque_can {
  constexpr opaque_can() noexcept = default;
  constexpr explicit opaque_can(const can_controller_key key) noexcept : m_key(key) {}

  can_controller_key m_key{can_controller_key::Invalid};
};
}  // namespace ru::driver
