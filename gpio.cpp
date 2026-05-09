#include "gpio.hpp"
#include "mapping.hpp"
#include "stm32h5xx_hal.h"

using namespace ru::driver;

namespace ru::driver {
namespace {
constexpr opaque_gpio make_opaque(const GpioId id) noexcept {
  switch (id) {
#define RU_STM32H5XX_GPIO_CASE(name, port, pin, active_state, mode, pull, speed)         \
    case GpioId::name:                                                                    \
      return opaque_gpio{port, pin, active_state, mode, pull, speed};
    RU_STM32H5XX_GPIO_MAP(RU_STM32H5XX_GPIO_CASE)
#undef RU_STM32H5XX_GPIO_CASE
    default:
      return opaque_gpio{};
  }
}
}  // namespace

Gpio::Gpio(const GpioId id) noexcept : m_id(id), m_opaque(make_opaque(id)) {
}

result Gpio::start() noexcept {
  return result::OK;
}

result Gpio::init() noexcept {
  return m_opaque.init();
}

result Gpio::stop() noexcept {
  return m_opaque.stop();
}

GpioValue Gpio::active_value() const noexcept {
  return m_opaque.active_high() ? GpioValue::HIGH : GpioValue::LOW;
}

GpioPolarity Gpio::polarity() const noexcept {
  return m_opaque.active_high() ? GpioPolarity::ACTIVE_HIGH : GpioPolarity::ACTIVE_LOW;
}

expected::expected<bool, result> Gpio::is_active() const noexcept {
  return m_opaque.is_active();
}

expected::expected<bool, result> Gpio::is_inactive() const noexcept {
  return !m_opaque.is_active();
}

expected::expected<bool, result> Gpio::is_high() const noexcept {
  return m_opaque.is_high();
}

expected::expected<bool, result> Gpio::is_low() const noexcept {
  return !m_opaque.is_high();
}

result Gpio::set_active() noexcept {
  return m_opaque.set_level(true);
}

result Gpio::set_inactive() noexcept {
  return m_opaque.set_level(false);
}

result Gpio::set_level(const bool active) noexcept {
  return active ? set_active() : set_inactive();
}

result Gpio::toggle() noexcept {
  return m_opaque.toggle();
}
}  // namespace ru::driver
