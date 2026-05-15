#include <array>

#include "timer.hpp"

#include "mapping.hpp"

using namespace ru::driver;

namespace ru::driver {
namespace {
const stm32h5xx::cfg::timer_config* config_for(const TimerId id) noexcept {
  switch (id) {
#define RU_STM32H5XX_TIMER_CONFIG(name, config) \
    case TimerId::name:                         \
      return &config;
    RU_STM32H5XX_TIMER_MAP(RU_STM32H5XX_TIMER_CONFIG)
#undef RU_STM32H5XX_TIMER_CONFIG
    default:
      return nullptr;
  }
}

std::array<TIM_HandleTypeDef, static_cast<std::size_t>(TimerId::COUNT)> g_handles{};

TIM_HandleTypeDef* timer_handle(const TimerId id) noexcept {
  const auto index = static_cast<std::size_t>(id);
  return index < g_handles.size() ? &g_handles[index] : nullptr;
}

opaque_timer make_opaque(const TimerId id) noexcept {
  return opaque_timer{timer_handle(id)};
}
}  // namespace

Timer::Timer(const TimerId id) noexcept : m_id(id), m_opaque(make_opaque(id)) {
}

result Timer::start() noexcept {
  return result::OK;
}

result Timer::init() noexcept {
  const auto* const config = config_for(m_id);
  if (config == nullptr) {
    return result::RECOVERABLE_ERROR;
  }

  auto* const p_handle = timer_handle(m_id);
  if (p_handle == nullptr) {
    return result::RECOVERABLE_ERROR;
  }

  if (p_handle->Instance != nullptr) {
    return result::OK;
  }

  return m_opaque.init(*config);
}

result Timer::stop() noexcept {
  return m_opaque.stop();
}

expected::expected<Timestamp, result> Timer::time_now() const noexcept {
  return m_opaque.time_now();
}
}  // namespace ru::driver
