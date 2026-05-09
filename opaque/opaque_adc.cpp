#include "opaque_adc.hpp"

#include "utils/common.hpp"

using namespace ru::driver;

namespace ru::driver {
result opaque_adc::init(ADC_HandleTypeDef* const p_handle) const noexcept {

  enable_adc_clock(m_p_instance);
  init_pin(m_p_port, m_pin, GPIO_MODE_ANALOG, GPIO_NOPULL);

  p_handle->Instance = m_p_instance;
  p_handle->Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  p_handle->Init.Resolution = ADC_RESOLUTION_12B;
  p_handle->Init.DataAlign = ADC_DATAALIGN_RIGHT;
  p_handle->Init.ScanConvMode = ADC_SCAN_DISABLE;
  p_handle->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  p_handle->Init.LowPowerAutoWait = DISABLE;
  p_handle->Init.ContinuousConvMode = DISABLE;
  p_handle->Init.NbrOfConversion = 1U;
  p_handle->Init.DiscontinuousConvMode = DISABLE;
  p_handle->Init.NbrOfDiscConversion = 1U;
  p_handle->Init.ExternalTrigConv = ADC_SOFTWARE_START;
  p_handle->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  p_handle->Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
  p_handle->Init.DMAContinuousRequests = DISABLE;
  p_handle->Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  p_handle->Init.OversamplingMode = DISABLE;

  if (HAL_ADC_Init(p_handle) != HAL_OK) {
    return result::RECOVERABLE_ERROR;
  }

  ADC_ChannelConfTypeDef channel{};
  channel.Channel = m_channel;
  channel.Rank = ADC_REGULAR_RANK_1;
  channel.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  channel.SingleDiff = ADC_SINGLE_ENDED;
  channel.OffsetNumber = ADC_OFFSET_NONE;
  channel.Offset = 0U;
  channel.OffsetSign = ADC_OFFSET_SIGN_NEGATIVE;
  channel.OffsetSaturation = DISABLE;

  return from_hal_status(HAL_ADC_ConfigChannel(p_handle, &channel));
}

result opaque_adc::stop(ADC_HandleTypeDef* const p_handle) const noexcept {
  (void)HAL_ADC_Stop(p_handle);
  return from_hal_status(HAL_ADC_DeInit(p_handle));
}

result opaque_adc::read(ADC_HandleTypeDef* const p_handle, uint16_t& r_value) const noexcept {
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

result opaque_adc::try_read(ADC_HandleTypeDef* const p_handle, bool& r_has_value,
                            uint16_t& r_value) const noexcept {
  r_has_value = false;

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
