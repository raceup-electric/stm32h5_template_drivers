#include <cstdint>

#include "rtc.hpp"

#include "mapping.hpp"
#include "stm32h5xx_hal_rtc.h"

using namespace ru::driver;

namespace ru::driver {
namespace {
constexpr Timestamp k_us_per_second = 1000000ULL;
constexpr int64_t k_seconds_per_day = 86400LL;

const stm32h5xx::cfg::rtc_config* config_for(const RtcId id) noexcept {
  switch (id) {
#define RU_STM32H5XX_RTC_CONFIG(name, config) \
    case RtcId::name:                         \
      return &config;
    RU_STM32H5XX_RTC_MAP(RU_STM32H5XX_RTC_CONFIG)
#undef RU_STM32H5XX_RTC_CONFIG
    default:
      return nullptr;
  }
}

result from_hal(const HAL_StatusTypeDef status) noexcept {
  switch (status) {
    case HAL_OK:
      return result::OK;
    case HAL_BUSY:
    case HAL_TIMEOUT:
      return result::RECOVERABLE_ERROR;
    default:
      return result::UNRECOVERABLE_ERROR;
  }
}

constexpr int64_t days_from_civil(const int64_t year, const unsigned month,
                                  const unsigned day) noexcept {
  const int64_t adjusted_year = year - (month <= 2U ? 1LL : 0LL);
  const int64_t era = (adjusted_year >= 0LL ? adjusted_year : adjusted_year - 399LL) / 400LL;
  const unsigned year_of_era = static_cast<unsigned>(adjusted_year - era * 400LL);
  const unsigned month_offset = month > 2U ? month - 3U : month + 9U;
  const unsigned day_of_year = (153U * month_offset + 2U) / 5U + day - 1U;
  const unsigned day_of_era = year_of_era * 365U + year_of_era / 4U -
                              year_of_era / 100U + day_of_year;
  return era * 146097LL + static_cast<int64_t>(day_of_era) - 719468LL;
}

void civil_from_days(const int64_t days, int64_t& year, unsigned& month,
                     unsigned& day) noexcept {
  const int64_t z = days + 719468LL;
  const int64_t era = (z >= 0LL ? z : z - 146096LL) / 146097LL;
  const unsigned day_of_era = static_cast<unsigned>(z - era * 146097LL);
  const unsigned year_of_era = (day_of_era - day_of_era / 1460U + day_of_era / 36524U -
                                day_of_era / 146096U) /
                               365U;
  year = static_cast<int64_t>(year_of_era) + era * 400LL;
  const unsigned day_of_year = day_of_era -
                               (365U * year_of_era + year_of_era / 4U -
                                year_of_era / 100U);
  const unsigned month_prime = (5U * day_of_year + 2U) / 153U;
  day = day_of_year - (153U * month_prime + 2U) / 5U + 1U;
  month = month_prime < 10U ? month_prime + 3U : month_prime - 9U;
  year += month <= 2U ? 1LL : 0LL;
}

Timestamp timestamp_from_calendar(const RTC_DateTypeDef& date,
                                  const RTC_TimeTypeDef& time) noexcept {
  const int64_t year = 2000LL + static_cast<int64_t>(date.Year);
  const int64_t days = days_from_civil(year, date.Month, date.Date);
  const int64_t seconds = days * k_seconds_per_day +
                          static_cast<int64_t>(time.Hours) * 3600LL +
                          static_cast<int64_t>(time.Minutes) * 60LL +
                          static_cast<int64_t>(time.Seconds);
  return seconds < 0LL ? 0ULL : static_cast<Timestamp>(seconds) * k_us_per_second;
}

void calendar_from_timestamp(const Timestamp value, RTC_DateTypeDef& date,
                             RTC_TimeTypeDef& time) noexcept {
  const int64_t seconds = static_cast<int64_t>(value / k_us_per_second);
  const int64_t days = seconds / k_seconds_per_day;
  const int64_t seconds_of_day = seconds % k_seconds_per_day;

  int64_t year = 0LL;
  unsigned month = 0U;
  unsigned day = 0U;
  civil_from_days(days, year, month, day);

  date.Year = static_cast<uint8_t>(year >= 2000LL ? year - 2000LL : 0LL);
  date.Month = static_cast<uint8_t>(month);
  date.Date = static_cast<uint8_t>(day);
  date.WeekDay = static_cast<uint8_t>(((days + 3LL) % 7LL) + 1LL);

  const int64_t normalized = seconds_of_day < 0LL ? seconds_of_day + k_seconds_per_day
                                                   : seconds_of_day;
  time.Hours = static_cast<uint8_t>(normalized / 3600LL);
  time.Minutes = static_cast<uint8_t>((normalized % 3600LL) / 60LL);
  time.Seconds = static_cast<uint8_t>(normalized % 60LL);
  time.TimeFormat = RTC_HOURFORMAT12_AM;
  time.SubSeconds = 0U;
  time.SecondFraction = 0U;
  time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  time.StoreOperation = RTC_STOREOPERATION_RESET;
}

RTC_HandleTypeDef handle_for(const stm32h5xx::cfg::rtc_config& config) noexcept {
  RTC_HandleTypeDef handle{};
  handle.Instance = config.instance();
  handle.Init = config.init;
  return handle;
}
}  // namespace

result Rtc::start() noexcept {
  return result::OK;
}

result Rtc::init() noexcept {
  const auto* const p_config = config_for(m_id);
  if (p_config == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  auto handle = handle_for(*p_config);
  return from_hal(HAL_RTC_Init(&handle));
}

result Rtc::stop() noexcept {
  const auto* const p_config = config_for(m_id);
  if (p_config == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  auto handle = handle_for(*p_config);
  return from_hal(HAL_RTC_DeInit(&handle));
}

expected::expected<Timestamp, result> Rtc::now() const noexcept {
  const auto* const p_config = config_for(m_id);
  if (p_config == nullptr) {
    return expected::unexpected(result::UNRECOVERABLE_ERROR);
  }

  auto handle = handle_for(*p_config);
  RTC_TimeTypeDef time{};
  RTC_DateTypeDef date{};
  if (HAL_RTC_GetTime(&handle, &time, RTC_FORMAT_BIN) != HAL_OK) {
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }

  if (HAL_RTC_GetDate(&handle, &date, RTC_FORMAT_BIN) != HAL_OK) {
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }

  return timestamp_from_calendar(date, time);
}

result Rtc::set(const Timestamp value) noexcept {
  const auto* const p_config = config_for(m_id);
  if (p_config == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  auto handle = handle_for(*p_config);
  RTC_TimeTypeDef time{};
  RTC_DateTypeDef date{};
  calendar_from_timestamp(value, date, time);

  if (HAL_RTC_SetTime(&handle, &time, RTC_FORMAT_BIN) != HAL_OK) {
    return result::RECOVERABLE_ERROR;
  }

  return from_hal(HAL_RTC_SetDate(&handle, &date, RTC_FORMAT_BIN));
}
}  // namespace ru::driver