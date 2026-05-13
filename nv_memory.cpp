#include "nv_memory.hpp"

#include "mapping.hpp"

using namespace ru::driver;

namespace ru::driver {
namespace {
const stm32h5xx::cfg::nv_memory_config* config_for(const NvMemoryId id) noexcept {
  switch (id) {
#define RU_STM32H5XX_NV_MEMORY_CONFIG(name, config) \
    case NvMemoryId::name:                          \
      return &config;
    RU_STM32H5XX_NV_MEMORY_MAP(RU_STM32H5XX_NV_MEMORY_CONFIG)
#undef RU_STM32H5XX_NV_MEMORY_CONFIG
    default:
      return nullptr;
  }
}

opaque_nv_memory make_opaque(const NvMemoryId id) noexcept {
  const auto* const config = config_for(id);
  return config != nullptr ? opaque_nv_memory{config->backend, config->arg0, config->size}
                           : opaque_nv_memory{};
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
