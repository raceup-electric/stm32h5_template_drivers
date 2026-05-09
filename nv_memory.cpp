#include "nv_memory.hpp"

#include "mapping.hpp"

using namespace ru::driver;

namespace ru::driver {
namespace {
struct NvMemoryConfig {
  NvMemoryBackend backend{NvMemoryBackend::None};
  uint32_t arg0{0U};
  uint32_t size{0U};
};

constexpr NvMemoryConfig make_config(const NvMemoryId id) noexcept {
  switch (id) {
#define RU_STM32H5XX_NV_MEMORY_CASE(name, backend_kind, arg0_value, size_value) \
    case NvMemoryId::name:                                                       \
      return {NvMemoryBackend::backend_kind, static_cast<uint32_t>(arg0_value),  \
              static_cast<uint32_t>(size_value)};
    RU_STM32H5XX_NV_MEMORY_MAP(RU_STM32H5XX_NV_MEMORY_CASE)
#undef RU_STM32H5XX_NV_MEMORY_CASE
    default:
      return {};
  }
}

opaque_nv_memory make_opaque(const NvMemoryId id) noexcept {
  const auto config = make_config(id);

  switch (config.backend) {
    case NvMemoryBackend::EmulatedEeprom:
      return opaque_nv_memory::make_emulated_eeprom(
          static_cast<uint16_t>(config.arg0), config.size);

    case NvMemoryBackend::FlashRegion:
      return opaque_nv_memory::make_flash_region(config.arg0, config.size);

    default:
      return {};
  }
}
}  // namespace

NvMemory::NvMemory(const NvMemoryId id) noexcept : m_id(id), m_opaque(make_opaque(id)) {
}

result NvMemory::start() noexcept {
  return result::OK;
}

result NvMemory::init() noexcept {
  return m_opaque.init();
}

result NvMemory::stop() noexcept {
  return m_opaque.stop();
}

uint32_t NvMemory::capacity() const noexcept {
  return m_opaque.capacity();
}

result NvMemory::clear() noexcept {
  return m_opaque.clear();
}

result NvMemory::read(const uint32_t address, uint8_t* const p_data,
                      const size_t len) noexcept {
  return m_opaque.read(address, p_data, len);
}

expected::expected<uint8_t, result> NvMemory::read(const uint32_t address) noexcept {
  uint8_t value{0U};
  const auto status = read(address, &value, sizeof(value));
  if (status != result::OK) {
    return expected::unexpected(status);
  }

  return value;
}

result NvMemory::write(const uint32_t address, const uint8_t* const p_data,
                       const size_t len) noexcept {
  return m_opaque.write(address, p_data, len);
}

result NvMemory::write(const uint32_t address, const uint8_t value) noexcept {
  return write(address, &value, sizeof(value));
}
}  // namespace ru::driver
