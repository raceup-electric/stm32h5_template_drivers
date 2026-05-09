#include "eeprom_itf_crc.h"

#include <cstdint>

namespace {
std::uint16_t crc16_poly8005_initffff(const std::uint8_t* const data,
                                      const std::uint16_t len) noexcept {
  std::uint16_t crc = 0xFFFFU;

  for (std::uint16_t index = 0U; index < len; ++index) {
    crc ^= static_cast<std::uint16_t>(data[index]) << 8U;
    for (std::uint8_t bit = 0U; bit < 8U; ++bit) {
      crc = (crc & 0x8000U) != 0U
                ? static_cast<std::uint16_t>((crc << 1U) ^ 0x8005U)
                : static_cast<std::uint16_t>(crc << 1U);
    }
  }

  return crc;
}
}  // namespace

extern "C" ee_itf_crc_status EE_ITF_CRC_Init(void* const crc_object) {
  static_cast<void>(crc_object);
  return EE_ITF_CRC_OK;
}

extern "C" ee_itf_crc_status EE_ITF_CRC_Calcul16Bit(
    std::uint8_t* const crc_data, const std::uint16_t data_size_byte,
    std::uint16_t* const crc_value) {
  if (crc_data == nullptr || crc_value == nullptr) {
    return EE_ITF_CRC_ERROR;
  }

  *crc_value = crc16_poly8005_initffff(crc_data, data_size_byte);
  return EE_ITF_CRC_OK;
}

extern "C" ee_itf_crc_status EE_ITF_CRC_Calcul8Bit(
    std::uint8_t* const crc_data, const std::uint16_t data_size_byte,
    std::uint8_t* const crc_value) {
  std::uint16_t crc16 = 0U;

  if (crc_data == nullptr || crc_value == nullptr) {
    return EE_ITF_CRC_ERROR;
  }

  if (EE_ITF_CRC_Calcul16Bit(crc_data, data_size_byte, &crc16) !=
      EE_ITF_CRC_OK) {
    return EE_ITF_CRC_ERROR;
  }

  *crc_value = static_cast<std::uint8_t>(crc16 & 0x00FFU);
  return EE_ITF_CRC_OK;
}
