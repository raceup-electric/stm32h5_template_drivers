#include <algorithm>
#include <array>

#include "pwm.hpp"
#include "mapping.hpp"

using namespace ru::driver;

namespace ru::driver {
namespace {
constexpr std::size_t pwm_index(const PwmId id) noexcept {
  return static_cast<std::size_t>(id);
}

TIM_HandleTypeDef& pwm_handle(const PwmId id) noexcept {
  static std::array<TIM_HandleTypeDef, static_cast<std::size_t>(PwmId::COUNT)> handles{};
  const auto index = pwm_index(id);
  return index < handles.size() ? handles[index] : handles.front();
}

constexpr opaque_pwm make_opaque(const PwmId id) noexcept {
  switch (id) {
#define RU_STM32H5XX_PWM_CASE(name, instance, port, pin, channel, alternate)             \
    case PwmId::name:                                                                     \
      return opaque_pwm{instance, port, pin, channel, alternate};
    RU_STM32H5XX_PWM_MAP(RU_STM32H5XX_PWM_CASE)
#undef RU_STM32H5XX_PWM_CASE
    default:
      return opaque_pwm{};
  }
}
}  // namespace

Pwm::Pwm(const PwmId id) noexcept : m_id(id), m_opaque(make_opaque(id)) {
}

result Pwm::start() noexcept {
  return result::OK;
}

result Pwm::init() noexcept {
  auto& handle = pwm_handle(m_id);
  if (handle.Instance != nullptr) {
    return result::OK;
  }

  return m_opaque.init(&handle, m_opaque.m_frequency_hz);
}

result Pwm::stop() noexcept {
  auto& handle = pwm_handle(m_id);
  const auto stop_status = m_opaque.stop(&handle);
  if (stop_status != result::OK) {
    return stop_status;
  }

  const auto deinit_status = m_opaque.deinit(&handle);
  if (deinit_status == result::OK) {
    handle.Instance = nullptr;
  }

  return deinit_status;
}

result Pwm::enable() noexcept {
  auto& handle = pwm_handle(m_id);

  if (handle.Instance == nullptr) {
    const auto init_status = m_opaque.init(&handle, m_opaque.m_frequency_hz);
    if (init_status != result::OK) {
      return init_status;
    }
  }

  return m_opaque.start(&handle);
}

result Pwm::disable() noexcept {
  return m_opaque.stop(&pwm_handle(m_id));
}

result Pwm::set_frequency(const uint32_t frequency_hz) noexcept {
  const auto clamped_frequency_hz = std::max<uint32_t>(frequency_hz, 1U);
  auto& handle = pwm_handle(m_id);
  const auto was_enabled = m_opaque.is_enabled(&handle);
  if (!was_enabled) {
    return m_opaque.init(&handle, clamped_frequency_hz);
  }

  const auto duty_cycle = m_opaque.get_duty_cycle(&handle);
  if (!duty_cycle.has_value()) {
    return duty_cycle.error();
  }

  const auto stop_status = m_opaque.stop(&handle);
  if (stop_status != result::OK) {
    return stop_status;
  }

  const auto init_status = m_opaque.init(&handle, clamped_frequency_hz);
  if (init_status != result::OK) {
    return init_status;
  }

  const auto duty_status = m_opaque.update_duty_cycle(&handle, duty_cycle.value());
  if (duty_status != result::OK) {
    return duty_status;
  }

  return m_opaque.start(&handle);
}

result Pwm::set_duty_cycle(const uint16_t duty_cycle_permille) noexcept {
  auto& handle = pwm_handle(m_id);
  const auto clamped_duty_cycle =
      static_cast<uint16_t>(std::min<uint16_t>(duty_cycle_permille, 1000U));
  if (handle.Instance == nullptr) {
    const auto init_status = m_opaque.init(&handle, m_opaque.m_frequency_hz);
    if (init_status != result::OK) {
      return init_status;
    }
  }

  return m_opaque.update_duty_cycle(&handle, clamped_duty_cycle);
}

expected::expected<uint16_t, result> Pwm::get_duty_cycle() const noexcept {
  return m_opaque.get_duty_cycle(&pwm_handle(m_id));
}
}  // namespace ru::driver
