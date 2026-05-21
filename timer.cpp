#include "timer.hpp"

#include "mapping.hpp"
#include "mapping_types.hpp"
#include "stm_common.hpp"

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

uint32_t timer_prescaler(const TIM_TypeDef* instance,
                         const uint32_t counter_frequency_hz) noexcept {
  const auto clock_hz = timer_input_clock_hz(instance);
  const auto target_counter_frequency_hz = counter_frequency_hz == 0U ? 1U : counter_frequency_hz;
  return clock_hz > target_counter_frequency_hz
             ? (clock_hz / target_counter_frequency_hz) - 1U
             : 0U;
}
}  // namespace

opaque_timer make_opaque(const TimerId id) noexcept {
  (void)id;
  return opaque_timer{};
}

Timer::Timer(const TimerId id) noexcept : m_id(id), m_opaque(make_opaque(id)) {
}

result Timer::start() noexcept {
  return result::OK;
}

result Timer::init() noexcept {
  const auto* const config = config_for(m_id);
  if (config == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  if (m_opaque.initialized()) {
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

bool opaque_timer::initialized() const noexcept {
  return m_handle.Instance != nullptr;
}

result opaque_timer::init(const stm32h5xx::cfg::timer_config& config) const noexcept {
  enable_tim_clock(config.instance());

  auto& r_handle = m_handle;
  r_handle.Instance = config.instance();
  r_handle.Init = config.init;
  const auto prescaler = timer_prescaler(config.instance(), config.counter_clock_hz);
  r_handle.Init.Prescaler = prescaler;

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

result opaque_timer::stop() const noexcept {
  if (m_handle.Instance == nullptr) {
    return result::OK;
  }

  auto& r_handle = m_handle;
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

expected::expected<uint64_t, result> opaque_timer::time_now() const noexcept {
  if (m_handle.Instance == nullptr) {
    return expected::unexpected(result::UNRECOVERABLE_ERROR);
  }

  const auto counter_clock_hz =
      timer_input_clock_hz(m_handle.Instance) /
      (static_cast<uint32_t>(m_handle.Instance->PSC) + 1U);
  if (counter_clock_hz == 0U) {
    return expected::unexpected(result::UNRECOVERABLE_ERROR);
  }

  const auto ticks = static_cast<uint64_t>(
      __HAL_TIM_GET_COUNTER(const_cast<TIM_HandleTypeDef*>(&m_handle)));
  return (ticks * 1000000ULL) / static_cast<uint64_t>(counter_clock_hz);
}
}  // namespace ru::driver
