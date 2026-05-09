#pragma once

#include <cstddef>
#include <cstdint>

#include "common.hpp"

namespace ru::driver {
class NvMemory;

enum class NvMemoryBackend : uint8_t { None, EmulatedEeprom, FlashRegion };

struct opaque_nv_memory {
 public:
  constexpr opaque_nv_memory() noexcept = default;

  static constexpr opaque_nv_memory make_emulated_eeprom(
      const uint16_t base_virtual_address, const uint32_t capacity) noexcept {
    opaque_nv_memory out{};
    out.m_backend = NvMemoryBackend::EmulatedEeprom;
    out.m_capacity = capacity;
    out.m_base_virtual_address = base_virtual_address;
    return out;
  }

  static constexpr opaque_nv_memory make_flash_region(const uint32_t offset,
                                                      const uint32_t capacity) noexcept {
    opaque_nv_memory out{};
    out.m_backend = NvMemoryBackend::FlashRegion;
    out.m_capacity = capacity;
    out.m_flash_offset = offset;
    return out;
  }

 private:
  friend class NvMemory;

  result init() noexcept;
  result stop() noexcept;
  uint32_t capacity() const noexcept { return m_capacity; }
  result clear() noexcept;
  result read(uint32_t address, uint8_t* p_data, size_t len) const noexcept;
  result write(uint32_t address, const uint8_t* p_data, size_t len) noexcept;

  NvMemoryBackend m_backend{NvMemoryBackend::None};
  uint32_t m_capacity{0U};
  uint32_t m_flash_offset{0U};
  uint16_t m_base_virtual_address{0U};
};
}  // namespace ru::driver
