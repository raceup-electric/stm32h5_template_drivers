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

result opaque_pwm::init(TIM_HandleTypeDef* const p_handle, const uint32_t frequency_hz) noexcept {
  auto& r_handle = *p_handle;

  enable_tim_clock(m_p_instance);
  init_pin(m_p_port, m_pin, GPIO_MODE_AF_PP, GPIO_PULLUP,
           GPIO_SPEED_FREQ_VERY_HIGH, m_alternate);

  const auto timer_clock_hz = SystemCoreClock == 0U ? 64000000U : SystemCoreClock;
  constexpr uint32_t k_counter_clock_hz = 1000000U;
  const auto target_frequency_hz = std::max<uint32_t>(frequency_hz, 1U);
  const auto prescaler =
      timer_clock_hz > k_counter_clock_hz ? (timer_clock_hz / k_counter_clock_hz) - 1U : 0U;
  const auto period_ticks = std::max<uint32_t>(k_counter_clock_hz / target_frequency_hz, 1U);
  const auto period = period_ticks - 1U;

  r_handle.Instance = m_p_instance;
  r_handle.Init.Prescaler = prescaler;
  r_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  r_handle.Init.Period = period;
  r_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  r_handle.Init.RepetitionCounter = 0U;
  r_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_PWM_Init(&r_handle) != HAL_OK) {
    return result::RECOVERABLE_ERROR;
  }

  TIM_OC_InitTypeDef config{};
  config.OCMode = TIM_OCMODE_PWM1;
  config.Pulse = 0U;
  config.OCPolarity = TIM_OCPOLARITY_HIGH;
  config.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  config.OCFastMode = TIM_OCFAST_DISABLE;
  config.OCIdleState = TIM_OCIDLESTATE_RESET;
  config.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  const auto status = from_hal_status(HAL_TIM_PWM_ConfigChannel(&r_handle, &config, m_channel));
  if (status == result::OK) {
    m_frequency_hz = target_frequency_hz;
  }

  return status;
}

result opaque_pwm::start(TIM_HandleTypeDef* const p_handle) noexcept {
  auto& r_handle = *p_handle;
  if (r_handle.Instance == nullptr) {
    return result::RECOVERABLE_ERROR;
  }

  return from_hal_status(HAL_TIM_PWM_Start(&r_handle, m_channel));
}

result opaque_pwm::stop(TIM_HandleTypeDef* const p_handle) noexcept {
  auto& r_handle = *p_handle;
  if (r_handle.Instance == nullptr) {
    return result::OK;
  }

  return from_hal_status(HAL_TIM_PWM_Stop(&r_handle, m_channel));
}

result opaque_pwm::deinit(TIM_HandleTypeDef* const p_handle) noexcept {
  auto& r_handle = *p_handle;
  if (r_handle.Instance == nullptr) {
    return result::OK;
  }

  return from_hal_status(HAL_TIM_PWM_DeInit(&r_handle));
}

bool opaque_pwm::is_enabled(const TIM_HandleTypeDef* const p_handle) const noexcept {
  const auto& r_handle = *p_handle;
  if (r_handle.Instance == nullptr) {
    return false;
  }

  return (r_handle.Instance->CCER & pwm_channel_enable_mask(m_channel)) != 0U;
}

result opaque_pwm::update_duty_cycle(TIM_HandleTypeDef* const p_handle,
                                     const uint16_t duty_cycle_permille) noexcept {
  auto& r_handle = *p_handle;
  if (r_handle.Instance == nullptr) {
    return result::OK;
  }

  __HAL_TIM_SET_COMPARE(&r_handle, m_channel,
                        pwm_pulse(r_handle.Instance->ARR, duty_cycle_permille));
  return result::OK;
}

expected::expected<uint16_t, result> opaque_pwm::get_duty_cycle(
    const TIM_HandleTypeDef* const p_handle) const noexcept {
  const auto& r_handle = *p_handle;
  if (r_handle.Instance == nullptr || !is_enabled(p_handle)) {
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }

  return pwm_duty_cycle_permille(r_handle.Instance->ARR,
                                 __HAL_TIM_GET_COMPARE(const_cast<TIM_HandleTypeDef*>(&r_handle), m_channel));
}
}  // namespace ru::driver
