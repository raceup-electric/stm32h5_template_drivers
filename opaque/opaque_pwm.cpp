#include <algorithm>

#include "opaque_pwm.hpp"

#include "utils/common.hpp"

using namespace ru::driver;

namespace ru::driver {
namespace {
constexpr uint32_t pwm_pulse(const uint32_t period, const uint16_t duty_cycle_permille) noexcept {
  const auto pulse = ((period + 1U) * duty_cycle_permille) / 1000U;
  return pulse > period ? period : pulse;
}

constexpr uint32_t pwm_channel_enable_mask(const uint32_t channel) noexcept {
  switch (channel) {
    case TIM_CHANNEL_1:
      return TIM_CCER_CC1E;
    case TIM_CHANNEL_2:
      return TIM_CCER_CC2E;
    case TIM_CHANNEL_3:
      return TIM_CCER_CC3E;
    case TIM_CHANNEL_4:
      return TIM_CCER_CC4E;
    default:
      return 0U;
  }
}

constexpr uint16_t pwm_duty_cycle_permille(const uint32_t period,
                                           const uint32_t compare_value) noexcept {
  if (period == 0U || compare_value == 0U) {
    return 0U;
  }

  if (compare_value >= period) {
    return 1000U;
  }

  return static_cast<uint16_t>((compare_value * 1000U) / (period + 1U));
}
}  // namespace

result opaque_pwm::init(const stm32h5xx::cfg::pwm_config& config,
                        const uint32_t frequency_hz) noexcept {
  if (m_p_handle == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  auto* const p_handle = m_p_handle;
  auto& r_handle = *p_handle;

  enable_tim_clock(config.instance());
  init_pin(config.port(), config.pin_init);

  const auto timer_clock_hz = SystemCoreClock == 0U ? 64000000U : SystemCoreClock;
  const auto target_frequency_hz = std::max<uint32_t>(frequency_hz, 1U);
  const auto prescaler =
      timer_clock_hz > config.counter_clock_hz
          ? (timer_clock_hz / config.counter_clock_hz) - 1U
          : 0U;
  const auto period_ticks =
      std::max<uint32_t>(config.counter_clock_hz / target_frequency_hz, 1U);
  const auto period = period_ticks - 1U;

  r_handle.Instance = config.instance();
  r_handle.Init = config.init;
  r_handle.Init.Prescaler = prescaler;
  r_handle.Init.Period = period;

  if (HAL_TIM_PWM_Init(&r_handle) != HAL_OK) {
    return result::RECOVERABLE_ERROR;
  }

  auto channel_config = config.channel_init;

  return from_hal_status(HAL_TIM_PWM_ConfigChannel(&r_handle, &channel_config, m_channel));
}

result opaque_pwm::enable() noexcept {
  if (m_p_handle == nullptr) {
    return result::RECOVERABLE_ERROR;
  }

  auto* const p_handle = m_p_handle;
  auto& r_handle = *p_handle;
  if (r_handle.Instance == nullptr) {
    return result::RECOVERABLE_ERROR;
  }

  return from_hal_status(HAL_TIM_PWM_Start(&r_handle, m_channel));
}

result opaque_pwm::disable() noexcept {
  if (m_p_handle == nullptr) {
    return result::RECOVERABLE_ERROR;
  }

  auto* const p_handle = m_p_handle;
  auto& r_handle = *p_handle;
  if (r_handle.Instance == nullptr) {
    return result::OK;
  }

  return from_hal_status(HAL_TIM_PWM_Stop(&r_handle, m_channel));
}

result opaque_pwm::deinit() noexcept {
  if (m_p_handle == nullptr) {
    return result::RECOVERABLE_ERROR;
  }

  auto* const p_handle = m_p_handle;
  auto& r_handle = *p_handle;
  if (r_handle.Instance == nullptr) {
    return result::OK;
  }

  return from_hal_status(HAL_TIM_PWM_DeInit(&r_handle));
}

bool opaque_pwm::is_enabled() const noexcept {
  if (m_p_handle == nullptr) {
    return false;
  }

  const auto* const p_handle = m_p_handle;
  const auto& r_handle = *p_handle;
  if (r_handle.Instance == nullptr) {
    return false;
  }

  return (r_handle.Instance->CCER & pwm_channel_enable_mask(m_channel)) != 0U;
}

result opaque_pwm::update_duty_cycle(const uint16_t duty_cycle_permille) noexcept {
  if (m_p_handle == nullptr) {
    return result::RECOVERABLE_ERROR;
  }

  auto* const p_handle = m_p_handle;
  auto& r_handle = *p_handle;
  if (r_handle.Instance == nullptr) {
    return result::OK;
  }

  __HAL_TIM_SET_COMPARE(&r_handle, m_channel,
                        pwm_pulse(r_handle.Instance->ARR, duty_cycle_permille));
  return result::OK;
}

expected::expected<uint16_t, result> opaque_pwm::get_duty_cycle() const noexcept {
  if (m_p_handle == nullptr) {
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }

  const auto* const p_handle = m_p_handle;
  const auto& r_handle = *p_handle;
  if (r_handle.Instance == nullptr || !is_enabled()) {
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }

  return pwm_duty_cycle_permille(r_handle.Instance->ARR,
                                 __HAL_TIM_GET_COMPARE(
                                     const_cast<TIM_HandleTypeDef*>(&r_handle), m_channel));
}
}  // namespace ru::driver
