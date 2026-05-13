#include <algorithm>
#include <array>

#include "pwm.hpp"
#include "mapping.hpp"

using namespace ru::driver;

namespace ru::driver {
namespace {
const stm32h5xx::cfg::pwm_config* config_for(const PwmId id) noexcept {
  switch (id) {
#define RU_STM32H5XX_PWM_CONFIG(name, config) \
    case PwmId::name:                         \
      return &config;
    RU_STM32H5XX_PWM_MAP(RU_STM32H5XX_PWM_CONFIG)
#undef RU_STM32H5XX_PWM_CONFIG
    default:
      return nullptr;
  }
}

constexpr std::size_t pwm_index(const PwmId id) noexcept {
  return static_cast<std::size_t>(id);
}

constexpr uint32_t k_default_frequency_hz = 1000U;

TIM_HandleTypeDef& pwm_handle(const PwmId id) noexcept {
  static std::array<TIM_HandleTypeDef, static_cast<std::size_t>(PwmId::COUNT)> handles{};
  const auto index = pwm_index(id);
  return index < handles.size() ? handles[index] : handles.front();
}

opaque_pwm make_opaque(const PwmId id) noexcept {
  const auto* const config = config_for(id);
  return config != nullptr ? opaque_pwm{&pwm_handle(id), config->channel} : opaque_pwm{};
}

}  // namespace

Pwm::Pwm(const PwmId id) noexcept : m_id(id), m_opaque(make_opaque(id)) {
}

result Pwm::start() noexcept {
  return result::OK;
}

result Pwm::init() noexcept {
  const auto* const config = config_for(m_id);
  if (config == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  auto& handle = pwm_handle(m_id);
  if (handle.Instance != nullptr) {
    return result::OK;
  }

  return m_opaque.init(*config, k_default_frequency_hz);
}

result Pwm::stop() noexcept {
  const auto* const config = config_for(m_id);
  if (config == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  auto& handle = pwm_handle(m_id);
  const auto stop_status = m_opaque.disable();
  if (stop_status != result::OK) {
    return stop_status;
  }

  const auto deinit_status = m_opaque.deinit();
  if (deinit_status == result::OK) {
    handle.Instance = nullptr;
  }

  return deinit_status;
}

result Pwm::enable() noexcept {
  return m_opaque.enable();
}

result Pwm::disable() noexcept {
  return m_opaque.disable();
}

result Pwm::set_frequency(const uint32_t frequency_hz) noexcept {
  const auto* const config = config_for(m_id);
  if (config == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  auto& handle = pwm_handle(m_id);
  if (handle.Instance == nullptr) {
    return result::RECOVERABLE_ERROR;
  }

  const auto clamped_frequency_hz = std::max<uint32_t>(frequency_hz, 1U);
  const auto was_enabled = m_opaque.is_enabled();
  const auto duty_cycle = m_opaque.get_duty_cycle();
  if (!duty_cycle.has_value()) {
    if (was_enabled) {
      return duty_cycle.error();
    }

    const auto stop_status = m_opaque.disable();
    if (stop_status != result::OK) {
      return stop_status;
    }

    const auto deinit_status = m_opaque.deinit();
    if (deinit_status != result::OK) {
      return deinit_status;
    }

    handle.Instance = nullptr;
    return m_opaque.init(*config, clamped_frequency_hz);
  }

  const auto stop_status = m_opaque.disable();
  if (stop_status != result::OK) {
    return stop_status;
  }

  const auto init_status = m_opaque.init(*config, clamped_frequency_hz);
  if (init_status != result::OK) {
    return init_status;
  }

  const auto duty_status = m_opaque.update_duty_cycle(duty_cycle.value());
  if (duty_status != result::OK) {
    return duty_status;
  }

  return m_opaque.enable();
}

result Pwm::set_duty_cycle(const uint16_t duty_cycle_permille) noexcept {
  const auto clamped_duty_cycle =
      static_cast<uint16_t>(std::min<uint16_t>(duty_cycle_permille, 1000U));
  if (pwm_handle(m_id).Instance == nullptr) {
    return result::RECOVERABLE_ERROR;
  }

  return m_opaque.update_duty_cycle(clamped_duty_cycle);
}

expected::expected<uint16_t, result> Pwm::get_duty_cycle() const noexcept {
  return m_opaque.get_duty_cycle();
}
}  // namespace ru::driver
