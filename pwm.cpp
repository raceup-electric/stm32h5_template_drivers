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

general_purpose_pwm_runtime_state& pwm_handle(const PwmId id) noexcept {
  static std::array<general_purpose_pwm_runtime_state,
                    static_cast<std::size_t>(PwmId::COUNT)>
      handles{};
  const auto index = pwm_index(id);
  return index < handles.size() ? handles[index] : handles.front();
}

low_power_pwm_runtime_state& lptim_pwm_state(const PwmId id) noexcept {
  static std::array<low_power_pwm_runtime_state,
                    static_cast<std::size_t>(PwmId::COUNT)>
      states{};
  const auto index = pwm_index(id);
  return index < states.size() ? states[index] : states.front();
}

bool pwm_initialized(const PwmId id, const stm32h5xx::cfg::pwm_config& config) noexcept {
  if (config.general_purpose() != nullptr) {
    return pwm_handle(id).p_handle != nullptr && pwm_handle(id).p_handle->Instance != nullptr;
  }

  if (config.low_power() != nullptr) {
    return lptim_pwm_state(id).handle.Instance != nullptr;
  }

  return false;
}

void reset_pwm_state(const PwmId id, const stm32h5xx::cfg::pwm_config& config) noexcept {
  if (config.general_purpose() != nullptr) {
    if (pwm_handle(id).p_handle != nullptr) {
      pwm_handle(id).p_handle->Instance = nullptr;
    }
  } else if (config.low_power() != nullptr) {
    lptim_pwm_state(id).handle.Instance = nullptr;
  }
}

opaque_pwm make_opaque(const PwmId id) noexcept {
  const auto* const config = config_for(id);
  if (config == nullptr) {
    return {};
  }

  if (const auto* const general_purpose = config->general_purpose();
      general_purpose != nullptr) {
    static std::array<TIM_HandleTypeDef, static_cast<std::size_t>(PwmId::COUNT)> tim_handles{};
    const auto index = pwm_index(id);
    auto& runtime = pwm_handle(id);
    runtime.p_handle = index < tim_handles.size() ? &tim_handles[index] : &tim_handles.front();
    return opaque_pwm{opaque_general_purpose_pwm{
        .p_state = &runtime,
        .channel = general_purpose->channel,
    }};
  }

  if (const auto* const low_power = config->low_power(); low_power != nullptr) {
    return opaque_pwm{opaque_low_power_pwm{
        .p_state = &lptim_pwm_state(id),
        .channel = low_power->channel,
    }};
  }

  return {};
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

  if (pwm_initialized(m_id, *config)) {
    return result::OK;
  }

  return m_opaque.init(*config, k_default_frequency_hz);
}

result Pwm::stop() noexcept {
  const auto* const config = config_for(m_id);
  if (config == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  const auto stop_status = m_opaque.disable();
  if (stop_status != result::OK) {
    return stop_status;
  }

  const auto deinit_status = m_opaque.deinit();
  if (deinit_status == result::OK) {
    reset_pwm_state(m_id, *config);
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

  if (!pwm_initialized(m_id, *config)) {
    return result::RECOVERABLE_ERROR;
  }

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

    reset_pwm_state(m_id, *config);
    return m_opaque.init(*config, frequency_hz);
  }

  const auto stop_status = m_opaque.disable();
  if (stop_status != result::OK) {
    return stop_status;
  }

  const auto init_status = m_opaque.init(*config, frequency_hz);
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
  const auto* const config = config_for(m_id);
  if (config == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  const auto clamped_duty_cycle =
      static_cast<uint16_t>(std::min<uint16_t>(duty_cycle_permille, 1000U));
  if (!pwm_initialized(m_id, *config)) {
    return result::RECOVERABLE_ERROR;
  }

  return m_opaque.update_duty_cycle(clamped_duty_cycle);
}

expected::expected<uint16_t, result> Pwm::get_duty_cycle() const noexcept {
  return m_opaque.get_duty_cycle();
}
}  // namespace ru::driver
