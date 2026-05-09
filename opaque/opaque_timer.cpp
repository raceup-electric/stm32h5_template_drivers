#include "opaque_timer.hpp"

#include <limits>

#include "utils/common.hpp"

using namespace ru::driver;

namespace ru::driver {
namespace {
constexpr uint32_t k_counter_frequency_hz = 1000000U;

constexpr uint32_t timer_period(const TIM_TypeDef* const p_instance) noexcept {
  if (p_instance == TIM2) {
    return std::numeric_limits<uint32_t>::max();
  }

#ifdef TIM5
  if (p_instance == TIM5) {
    return std::numeric_limits<uint32_t>::max();
  }
#endif

  return std::numeric_limits<uint16_t>::max();
}

uint32_t timer_prescaler() noexcept {
  const auto timer_clock_hz = SystemCoreClock == 0U ? 64000000U : SystemCoreClock;
  return timer_clock_hz > k_counter_frequency_hz
             ? (timer_clock_hz / k_counter_frequency_hz) - 1U
             : 0U;
}
}  // namespace

result opaque_timer::init(TIM_HandleTypeDef* const p_handle) const noexcept {
  if (p_handle == nullptr || m_p_instance == nullptr) {
    return result::RECOVERABLE_ERROR;
  }

  enable_tim_clock(m_p_instance);

  auto& r_handle = *p_handle;
  r_handle.Instance = m_p_instance;
  r_handle.Init.Prescaler = timer_prescaler();
  r_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  r_handle.Init.Period = timer_period(m_p_instance);
  r_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  r_handle.Init.RepetitionCounter = 0U;
  r_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  const auto init_status = HAL_TIM_Base_Init(&r_handle);
  if (init_status != HAL_OK) {
    r_handle.Instance = nullptr;
    return from_hal_status(init_status);
  }

  const auto start_status = HAL_TIM_Base_Start(&r_handle);
  if (start_status != HAL_OK) {
    (void)HAL_TIM_Base_DeInit(&r_handle);
    r_handle.Instance = nullptr;
    return from_hal_status(start_status);
  }

  __HAL_TIM_SET_COUNTER(&r_handle, 0U);
  return result::OK;
}

result opaque_timer::stop(TIM_HandleTypeDef* const p_handle) const noexcept {
  if (p_handle == nullptr || p_handle->Instance == nullptr) {
    return result::OK;
  }

  auto& r_handle = *p_handle;
  const auto stop_status = HAL_TIM_Base_Stop(&r_handle);
  if (stop_status != HAL_OK) {
    return from_hal_status(stop_status);
  }

  const auto deinit_status = HAL_TIM_Base_DeInit(&r_handle);
  if (deinit_status == HAL_OK) {
    r_handle.Instance = nullptr;
  }

  return from_hal_status(deinit_status);
}

expected::expected<uint64_t, result> opaque_timer::time_now(
    const TIM_HandleTypeDef* const p_handle) const noexcept {
  if (p_handle == nullptr || p_handle->Instance == nullptr) {
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }

  return static_cast<uint64_t>(
      __HAL_TIM_GET_COUNTER(const_cast<TIM_HandleTypeDef*>(p_handle)));
}
}  // namespace ru::driver
