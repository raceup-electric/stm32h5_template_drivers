#include "opaque_timer.hpp"

#include <limits>

#include "utils/common.hpp"

using namespace ru::driver;

namespace ru::driver {
namespace {
uint32_t timer_clock_hz() noexcept {
  return SystemCoreClock == 0U ? 64000000U : SystemCoreClock;
}

uint32_t timer_prescaler(const uint32_t counter_frequency_hz) noexcept {
  const auto clock_hz = timer_clock_hz();
  const auto target_counter_frequency_hz = counter_frequency_hz == 0U ? 1U : counter_frequency_hz;
  return clock_hz > target_counter_frequency_hz
             ? (clock_hz / target_counter_frequency_hz) - 1U
             : 0U;
}
}  // namespace

result opaque_timer::init(const stm32h5xx::cfg::timer_config& config) const noexcept {
  if (m_p_handle == nullptr) {
    return result::RECOVERABLE_ERROR;
  }

  auto* const p_handle = m_p_handle;
  enable_tim_clock(config.instance());

  auto& r_handle = *p_handle;
  r_handle.Instance = config.instance();
  r_handle.Init = config.init;
  const auto prescaler = timer_prescaler(config.counter_clock_hz);
  r_handle.Init.Prescaler = prescaler;
  m_counter_clock_hz = timer_clock_hz() / (prescaler + 1U);

  const auto init_status = HAL_TIM_Base_Init(&r_handle);
  if (init_status != HAL_OK) {
    m_counter_clock_hz = 0U;
    r_handle.Instance = nullptr;
    return from_hal_status(init_status);
  }

  const auto start_status = HAL_TIM_Base_Start(&r_handle);
  if (start_status != HAL_OK) {
    m_counter_clock_hz = 0U;
    (void)HAL_TIM_Base_DeInit(&r_handle);
    r_handle.Instance = nullptr;
    return from_hal_status(start_status);
  }

  __HAL_TIM_SET_COUNTER(&r_handle, 0U);
  return result::OK;
}

result opaque_timer::stop() const noexcept {
  if (m_p_handle == nullptr) {
    return result::RECOVERABLE_ERROR;
  }

  auto* const p_handle = m_p_handle;
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
    m_counter_clock_hz = 0U;
    r_handle.Instance = nullptr;
  }

  return from_hal_status(deinit_status);
}

expected::expected<uint64_t, result> opaque_timer::time_now() const noexcept {
  if (m_p_handle == nullptr) {
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }

  const auto* const p_handle = m_p_handle;
  if (p_handle->Instance == nullptr || m_counter_clock_hz == 0U) {
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }

  const auto ticks = static_cast<uint64_t>(
      __HAL_TIM_GET_COUNTER(const_cast<TIM_HandleTypeDef*>(p_handle)));
  return (ticks * 1000000ULL) / static_cast<uint64_t>(m_counter_clock_hz);
}
}  // namespace ru::driver
