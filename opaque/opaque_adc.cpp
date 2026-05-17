#include "opaque_adc.hpp"

#include "mapping_types.hpp"
#include "stm_common.hpp"

using namespace ru::driver;

namespace ru::driver {

void opaque_adc::start() noexcept{
  __HAL_RCC_ADC_CLK_ENABLE();
}

result opaque_adc::init(const stm32h5xx::cfg::adc_config& config) const noexcept {
  const auto* const polling = config.polling();
  if (m_p_handle == nullptr || polling == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  init_pin(config.port(), config.pin_init);

  auto* const p_handle = m_p_handle;
  p_handle->Instance = config.instance();
  p_handle->Init = polling->init;

  if (HAL_ADC_Init(p_handle) != HAL_OK) {
    return result::RECOVERABLE_ERROR;
  }

  auto channel = polling->channel_init;

  return from_hal_status(HAL_ADC_ConfigChannel(p_handle, &channel));
}

result opaque_adc::stop() const noexcept {
  if (m_p_handle == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  auto* const p_handle = m_p_handle;
  (void)HAL_ADC_Stop(p_handle);
  return from_hal_status(HAL_ADC_DeInit(p_handle));
}

result opaque_adc::read(uint16_t& r_value) const noexcept {
  if (m_p_handle == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  auto* const p_handle = m_p_handle;
  if (HAL_ADC_Start(p_handle) != HAL_OK) {
    return result::RECOVERABLE_ERROR;
  }

  if (HAL_ADC_PollForConversion(p_handle, HAL_MAX_DELAY) != HAL_OK) {
    (void)HAL_ADC_Stop(p_handle);
    return result::RECOVERABLE_ERROR;
  }

  r_value = static_cast<uint16_t>(HAL_ADC_GetValue(p_handle));
  (void)HAL_ADC_Stop(p_handle);
  return result::OK;
}

result opaque_adc::try_read(bool& r_has_value, uint16_t& r_value) const noexcept {
  r_has_value = false;
  if (m_p_handle == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  auto* const p_handle = m_p_handle;

  if (HAL_ADC_Start(p_handle) != HAL_OK) {
    return result::RECOVERABLE_ERROR;
  }

  const auto poll_status = HAL_ADC_PollForConversion(p_handle, 0U);
  if (poll_status == HAL_TIMEOUT) {
    (void)HAL_ADC_Stop(p_handle);
    return result::OK;
  }

  if (poll_status != HAL_OK) {
    (void)HAL_ADC_Stop(p_handle);
    return result::RECOVERABLE_ERROR;
  }

  r_has_value = true;
  r_value = static_cast<uint16_t>(HAL_ADC_GetValue(p_handle));
  (void)HAL_ADC_Stop(p_handle);
  return result::OK;
}
}  // namespace ru::driver
