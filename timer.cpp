#include <array>

#include "timer.hpp"

#include "mapping.hpp"
#include "utils/common.hpp"

using namespace ru::driver;

namespace ru::driver {
namespace {
std::array<TIM_HandleTypeDef, static_cast<std::size_t>(TimerId::COUNT)> g_handles{};

TIM_HandleTypeDef* timer_handle(const TimerId id) noexcept {
  const auto index = static_cast<std::size_t>(id);
  return index < g_handles.size() ? &g_handles[index] : nullptr;
}

constexpr opaque_timer make_opaque(const TimerId id) noexcept {
  switch (id) {
#define RU_STM32H5XX_TIMER_CASE(name, instance) \
    case TimerId::name:                         \
      return opaque_timer{instance};
    RU_STM32H5XX_TIMER_MAP(RU_STM32H5XX_TIMER_CASE)
#undef RU_STM32H5XX_TIMER_CASE
    default:
      return opaque_timer{};
  }
}
}  // namespace

Timer::Timer(const TimerId id) noexcept : m_id(id), m_opaque(make_opaque(id)) {
}

result Timer::start() noexcept {
  return result::OK;
}

result Timer::init() noexcept {
  auto* const p_handle = timer_handle(m_id);
  if (p_handle == nullptr) {
    return result::RECOVERABLE_ERROR;
  }

  if (p_handle->Instance != nullptr) {
    return result::OK;
  }

  return m_opaque.init(p_handle);
}

result Timer::stop() noexcept {
  return m_opaque.stop(timer_handle(m_id));
}

expected::expected<Timestamp, result> Timer::time_now() const noexcept {
  return m_opaque.time_now(timer_handle(m_id));
}
}  // namespace ru::driver
