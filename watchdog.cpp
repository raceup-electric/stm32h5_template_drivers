#include "watchdog.hpp"

using namespace ru::driver;

namespace ru::driver {
namespace {
constexpr uint32_t k_default_watchdog_timeout_ms = 1000U;
}  // namespace

Watchdog::Watchdog(const WatchdogId id) noexcept : m_id(id), m_opaque() {
}

result Watchdog::start() noexcept {
  return result::OK;
}

result Watchdog::init() noexcept {
  return m_opaque.init(k_default_watchdog_timeout_ms);
}

result Watchdog::stop() noexcept {
  return m_opaque.stop();
}

result Watchdog::kick() noexcept {
  return m_opaque.kick();
}

Timestamp Watchdog::timeout() const noexcept {
  return static_cast<Timestamp>(m_opaque.m_timeout_ms) * 1000U;
}
}  // namespace ru::driver
