#include <algorithm>
#include <limits>

#include "spi_proto.hpp"

#include "mapping.hpp"
#include "stm_common.hpp"
#include "stm32h5xx_hal_spi.h"

using namespace ru::driver;

namespace ru::driver {
namespace {
const stm32h5xx::cfg::spi_proto_config* config_for(const SpiProtoId id) noexcept {
  switch (id) {
#define RU_STM32H5XX_SPI_PROTO_CONFIG(name, config) \
    case SpiProtoId::name:                         \
      return &config;
    RU_STM32H5XX_SPI_PROTO_MAP(RU_STM32H5XX_SPI_PROTO_CONFIG)
#undef RU_STM32H5XX_SPI_PROTO_CONFIG
    default:
      return nullptr;
  }
}

SPI_HandleTypeDef handle_for(const stm32h5xx::cfg::spi_proto_config& config) noexcept {
  SPI_HandleTypeDef handle{};
  handle.Instance = config.instance();
  handle.Init = config.init;
  return handle;
}
}  // namespace

result SpiProto::start() noexcept {
  return result::OK;
}

result SpiProto::init() noexcept {
  const auto* const p_config = config_for(m_id);
  if (p_config == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  auto handle = handle_for(*p_config);
  return from_hal_status(HAL_SPI_Init(&handle));
}

result SpiProto::stop() noexcept {
  const auto* const p_config = config_for(m_id);
  if (p_config == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  auto handle = handle_for(*p_config);
  return from_hal_status(HAL_SPI_DeInit(&handle));
}

result SpiProto::transfer(const uint8_t* tx_data, uint8_t* rx_data,
                          const size_t len, const Timestamp timeout_uS) noexcept {
  if (len != 0U && (tx_data == nullptr || rx_data == nullptr)) {
    return result::UNRECOVERABLE_ERROR;
  }

  const auto* const p_config = config_for(m_id);
  if (p_config == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  auto handle = handle_for(*p_config);
  const uint32_t timeout_ms = std::max<uint32_t>(us_to_ms(timeout_uS), 1U);
  size_t remaining = len;
  while (remaining != 0U) {
    const uint16_t chunk = static_cast<uint16_t>(std::min<size_t>(
        remaining, static_cast<size_t>(std::numeric_limits<uint16_t>::max())));
    const auto status = HAL_SPI_TransmitReceive(&handle, tx_data, rx_data, chunk, timeout_ms);
    if (status != HAL_OK) {
      return from_hal_status(status);
    }

    remaining -= chunk;
    tx_data += chunk;
    rx_data += chunk;
  }

  return result::OK;
}
}  // namespace ru::driver