#include <algorithm>
#include <limits>

#include "sdmmc.hpp"

#include "mapping.hpp"
#include "stm32h5xx_hal_sd.h"

using namespace ru::driver;

namespace ru::driver {
namespace {
const stm32h5xx::cfg::sdmmc_config* config_for(const SdmmcId id) noexcept {
  switch (id) {
#define RU_STM32H5XX_SDMMC_CONFIG(name, config) \
    case SdmmcId::name:                         \
      return &config;
    RU_STM32H5XX_SDMMC_MAP(RU_STM32H5XX_SDMMC_CONFIG)
#undef RU_STM32H5XX_SDMMC_CONFIG
    default:
      return nullptr;
  }
}

result from_hal(const HAL_StatusTypeDef status) noexcept {
  switch (status) {
    case HAL_OK:
      return result::OK;
    case HAL_BUSY:
    case HAL_TIMEOUT:
      return result::RECOVERABLE_ERROR;
    default:
      return result::UNRECOVERABLE_ERROR;
  }
}

SD_HandleTypeDef handle_for(const stm32h5xx::cfg::sdmmc_config& config) noexcept {
  SD_HandleTypeDef handle{};
  handle.Instance = config.instance();
  handle.Init = config.init;
  return handle;
}
}  // namespace

result Sdmmc::start() noexcept {
  return result::OK;
}

result Sdmmc::init() noexcept {
  const auto* const p_config = config_for(m_id);
  if (p_config == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  auto handle = handle_for(*p_config);
  return from_hal(HAL_SD_Init(&handle));
}

result Sdmmc::stop() noexcept {
  const auto* const p_config = config_for(m_id);
  if (p_config == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  auto handle = handle_for(*p_config);
  return from_hal(HAL_SD_DeInit(&handle));
}

result Sdmmc::read_blocks(const uint32_t block, uint8_t* data,
                          const size_t block_count) noexcept {
  if (block_count != 0U && data == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  const auto* const p_config = config_for(m_id);
  if (p_config == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  auto handle = handle_for(*p_config);
  const uint32_t timeout_ms = p_config->timeout_ms;
  size_t remaining = block_count;
  uint32_t current_block = block;
  while (remaining != 0U) {
    const uint32_t chunk = static_cast<uint32_t>(std::min<size_t>(
        remaining, static_cast<size_t>(std::numeric_limits<uint32_t>::max())));
    const auto status = HAL_SD_ReadBlocks(&handle, data, current_block, chunk, timeout_ms);
    if (status != HAL_OK) {
      return from_hal(status);
    }

    remaining -= chunk;
    current_block += chunk;
    data += static_cast<size_t>(chunk) * 512U;
  }

  return result::OK;
}

result Sdmmc::write_blocks(const uint32_t block, const uint8_t* data,
                           const size_t block_count) noexcept {
  if (block_count != 0U && data == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  const auto* const p_config = config_for(m_id);
  if (p_config == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  auto handle = handle_for(*p_config);
  const uint32_t timeout_ms = p_config->timeout_ms;
  size_t remaining = block_count;
  uint32_t current_block = block;
  while (remaining != 0U) {
    const uint32_t chunk = static_cast<uint32_t>(std::min<size_t>(
        remaining, static_cast<size_t>(std::numeric_limits<uint32_t>::max())));
    const auto status = HAL_SD_WriteBlocks(&handle, const_cast<uint8_t*>(data),
                                           current_block, chunk, timeout_ms);
    if (status != HAL_OK) {
      return from_hal(status);
    }

    remaining -= chunk;
    current_block += chunk;
    data += static_cast<size_t>(chunk) * 512U;
  }

  return result::OK;
}
}  // namespace ru::driver