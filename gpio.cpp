#include "gpio.hpp"
#include "mapping.hpp"

using namespace ru::driver;

namespace ru::driver {
namespace {
const stm32h5xx::cfg::gpio_config* config_for(const GpioId id) noexcept {
  switch (id) {
#define RU_STM32H5XX_GPIO_CONFIG(name, config) \
    case GpioId::name:                         \
      return &config;
    RU_STM32H5XX_GPIO_MAP(RU_STM32H5XX_GPIO_CONFIG)
#undef RU_STM32H5XX_GPIO_CONFIG
    default:
      return nullptr;
  }
}

opaque_gpio make_opaque(const GpioId id) noexcept {
  const auto* const config = config_for(id);
  if (config == nullptr) {
    return opaque_gpio{};
  }

  return opaque_gpio{config->port(), config->pin(), config->active_high};
}

}  // namespace

Gpio::Gpio(const GpioId id) noexcept : m_id(id), m_opaque(make_opaque(id)) {
}

result Gpio::start() noexcept {
  return result::OK;
}

result Gpio::init() noexcept {
  const auto* const config = config_for(m_id);
  if (config == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  return m_opaque.init(config->init);
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
  const auto active = is_active();
  if (!active.has_value()) {
    return expected::unexpected(active.error());
  }

  return !active.value();
}

expected::expected<bool, result> Gpio::is_high() const noexcept {
  return m_opaque.is_high();
}

expected::expected<bool, result> Gpio::is_low() const noexcept {
  const auto high = is_high();
  if (!high.has_value()) {
    return expected::unexpected(high.error());
  }

  return !high.value();
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
