#include "eeprom_itf_flash.h"

#include <array>
#include <cstdint>

#include "eeprom_emul_conf.h"

namespace {
constexpr std::uint32_t k_program_size_bytes = 16UL;
constexpr std::uint32_t k_program_alignment = 16UL;

alignas(k_program_alignment) std::array<std::uint64_t, 2U> g_program_buffer{};

bool valid_program_request(const std::uint32_t address, const std::uint8_t* const data,
                           const std::uint16_t size) noexcept {
  return data != nullptr && size == k_program_size_bytes &&
         (address % k_program_alignment) == 0UL;
}

std::uint32_t bank_from_address(const std::uint32_t address) noexcept {
#if defined(FLASH_BANK_2)
  const auto bank_swapped =
      READ_BIT(FLASH->OPTSR_CUR, FLASH_OPTSR_SWAP_BANK) != 0U;
  const auto in_bank1_space = address < (FLASH_BASE + FLASH_BANK_SIZE);

  if (!bank_swapped) {
    return in_bank1_space ? FLASH_BANK_1 : FLASH_BANK_2;
  }

  return in_bank1_space ? FLASH_BANK_2 : FLASH_BANK_1;
#else
  static_cast<void>(address);
  return FLASH_BANK_1;
#endif
}

std::uint32_t sector_from_address(const std::uint32_t address) noexcept {
#if defined(FLASH_BANK_2)
  if (address < (FLASH_BASE + FLASH_BANK_SIZE)) {
    return (address - FLASH_BASE) / EE_FLASH_PAGE_SIZE;
  }

  return (address - (FLASH_BASE + FLASH_BANK_SIZE)) / EE_FLASH_PAGE_SIZE;
#else
  return (address - FLASH_BASE) / EE_FLASH_PAGE_SIZE;
#endif
}

void copy_to_program_buffer(const std::uint8_t* const data) noexcept {
  auto* const dst = reinterpret_cast<std::uint8_t*>(g_program_buffer.data());

  for (std::uint32_t index = 0UL; index < k_program_size_bytes; ++index) {
    dst[index] = data[index];
  }
}

ee_itf_flash_status lock_after_operation(const HAL_StatusTypeDef operation_status,
                                         const ee_itf_flash_status error_status) noexcept {
  const auto lock_status = HAL_FLASH_Lock();
  if (operation_status != HAL_OK || lock_status != HAL_OK) {
    return error_status;
  }

  return EE_ITF_FLASH_OK;
}
}  // namespace

extern "C" ee_itf_flash_status EE_ITF_FLASH_Init(
    void* const f_object, const ee_itf_flash_callback_t ee_callback) {
  static_cast<void>(f_object);
  static_cast<void>(ee_callback);
  return EE_ITF_FLASH_OK;
}

extern "C" ee_itf_flash_status EE_ITF_FLASH_WriteData(
    const std::uint32_t address, std::uint8_t* const p_data,
    const std::uint16_t size) {
  if (!valid_program_request(address, p_data, size)) {
    return EE_ITF_FLASH_ERROR_WRITE;
  }

  copy_to_program_buffer(p_data);

  if (HAL_FLASH_Unlock() != HAL_OK) {
    return EE_ITF_FLASH_ERROR_WRITE;
  }

  const auto data_address = static_cast<std::uint32_t>(
      reinterpret_cast<std::uintptr_t>(g_program_buffer.data()));
  const auto status =
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, address, data_address);

  return lock_after_operation(status, EE_ITF_FLASH_ERROR_WRITE);
}

extern "C" ee_itf_flash_status EE_ITF_FLASH_ReadData(
    const std::uint32_t address, std::uint8_t* const p_data,
    const std::uint16_t size) {
  if (p_data == nullptr) {
    return EE_ITF_FLASH_ERROR;
  }

  const auto* const source =
      reinterpret_cast<const std::uint8_t*>(static_cast<std::uintptr_t>(address));
  for (std::uint16_t index = 0U; index < size; ++index) {
    p_data[index] = source[index];
  }

  if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_ECCD) != 0U) {
    return EE_ITF_FLASH_ERROR_ECCC;
  }

  return EE_ITF_FLASH_OK;
}

extern "C" ee_itf_flash_status EE_ITF_FLASH_PageErase(
    const std::uint32_t address, const std::uint16_t nb_pages) {
  if (address < FLASH_BASE || nb_pages == 0U) {
    return EE_ITF_FLASH_ERROR_ERASE;
  }

  FLASH_EraseInitTypeDef erase{};
  std::uint32_t sector_error = 0UL;

  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Banks = bank_from_address(address);
  erase.Sector = sector_from_address(address);
  erase.NbSectors = nb_pages;

  if (HAL_FLASH_Unlock() != HAL_OK) {
    return EE_ITF_FLASH_ERROR_ERASE;
  }

  const auto status = HAL_FLASHEx_Erase(&erase, &sector_error);
  return lock_after_operation(status, EE_ITF_FLASH_ERROR_ERASE);
}

extern "C" ee_itf_flash_status EE_ITF_FLASH_PageErase_IT(
    const std::uint32_t address, const std::uint16_t nb_pages) {
  static_cast<void>(address);
  static_cast<void>(nb_pages);
  return EE_ITF_FLASH_NOTSUPPORTED;
}

extern "C" ee_itf_flash_status EE_ITF_FLASH_GetLastOperationStatus(
    std::uint32_t* const address) {
  if (address != nullptr) {
    *address = 0UL;
  }

  return EE_ITF_FLASH_OK;
}

extern "C" ee_itf_flash_status EE_ITF_FLASH_ClearError(void) {
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ECCD | FLASH_FLAG_ECCC | FLASH_FLAG_WRPERR |
                         FLASH_FLAG_PGSERR | FLASH_FLAG_STRBERR | FLASH_FLAG_INCERR);

  return EE_ITF_FLASH_OK;
}
