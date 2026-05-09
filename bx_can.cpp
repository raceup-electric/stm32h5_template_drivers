#include "can/bx_can.hpp"

#include "can_internal.hpp"

using namespace ru::driver;

namespace ru::driver {
using namespace can_internal;

expected::expected<CanMessage, result> Bx_canRx::read(Bx_fifo fifo) noexcept {
  return const_cast<Bx_can&>(m_can).read(fifo);
}

expected::expected<CanMessage, result> Bx_canRx::try_read(Bx_fifo fifo) noexcept {
  return const_cast<Bx_can&>(m_can).try_read(fifo);
}

result Bx_canTx::write(const CanMessage& message) noexcept {
  return const_cast<Bx_can&>(m_can).write(message);
}

result Bx_canTx::try_write(const CanMessage& message) noexcept {
  return const_cast<Bx_can&>(m_can).try_write(message);
}

Bx_can::Bx_can(const Bx_canId id) noexcept : m_id(id), m_opaque(make_opaque(id)) {
}

result Bx_can::start() noexcept {
  return result::OK;
}

result Bx_can::init() noexcept {
  return init_controller(m_opaque);
}

result Bx_can::stop() noexcept {
  return stop_controller(m_opaque);
}

expected::expected<CanMessage, result> Bx_can::read(Bx_fifo fifo) noexcept {
  const auto message = read_fifo_message(m_opaque, to_m_fifo(fifo));
  if (!message.has_value()) {
    return expected::unexpected(message.error());
  }

  return message.value().message;
}

expected::expected<CanMessage, result> Bx_can::try_read(Bx_fifo fifo) noexcept {
  return read(fifo);
}

result Bx_can::write(const CanMessage& message) noexcept {
  return write_controller_message(m_opaque, message);
}

result Bx_can::try_write(const CanMessage& message) noexcept {
  if (!initialized(m_opaque) || !started(m_opaque)) {
    return result::RECOVERABLE_ERROR;
  }

  if (HAL_FDCAN_GetTxFifoFreeLevel(&handle(m_opaque)) == 0U) {
    return result::RECOVERABLE_ERROR;
  }

  return write(message);
}

result Bx_can::set_priority(Bx_fifo fifo, uint8_t priority) {
  rx_priorities(m_opaque)[fifo_index(fifo)] = priority;
  return refresh_notifications(m_opaque);
}

expected::expected<uint8_t, result> Bx_can::get_priority(Bx_fifo fifo) {
  return rx_priorities(m_opaque)[fifo_index(fifo)];
}

result Bx_can::set_interrupt(Bx_fifo fifo, bool on) {
  rx_interrupts(m_opaque)[fifo_index(fifo)] = on;
  return refresh_notifications(m_opaque);
}

expected::expected<uint8_t, result> Bx_can::is_interrupt_on(Bx_fifo fifo) {
  return static_cast<uint8_t>(rx_interrupts(m_opaque)[fifo_index(fifo)]);
}

result Bx_can::set_error_priority(uint8_t priority) {
  error_priority(m_opaque) = priority;
  return refresh_notifications(m_opaque);
}

expected::expected<uint8_t, result> Bx_can::get_error_priority() {
  return error_priority(m_opaque);
}

result Bx_can::set_error_interrupt(bool on) {
  error_interrupt(m_opaque) = on;
  return refresh_notifications(m_opaque);
}

expected::expected<uint8_t, result> Bx_can::is_error_interrupt_on() {
  return static_cast<uint8_t>(error_interrupt(m_opaque));
}

result Bx_can::set_filter(Bx_filter filter, uint8_t id) {
  if (id >= k_std_filter_slots) {
    return result::UNRECOVERABLE_ERROR;
  }

  const auto status = with_stopped_controller(
      m_opaque, [&]() noexcept { return configure_bx_filter(m_opaque, filter, id, true); });
  if (status != result::OK) {
    return status;
  }

  bx_filters(m_opaque)[id] = filter;
  bx_filter_enabled(m_opaque)[id] = true;
  return result::OK;
}

result Bx_can::enable_filter(uint8_t id) {
  if (id >= k_std_filter_slots) {
    return result::UNRECOVERABLE_ERROR;
  }
  if (!bx_filters(m_opaque)[id].has_value()) {
    return result::RECOVERABLE_ERROR;
  }

  const auto status = with_stopped_controller(m_opaque, [&]() noexcept {
    return configure_bx_filter(m_opaque, bx_filters(m_opaque)[id].value(), id, true);
  });
  if (status != result::OK) {
    return status;
  }

  bx_filter_enabled(m_opaque)[id] = true;
  return result::OK;
}

result Bx_can::disable_filter(uint8_t id) {
  if (id >= k_std_filter_slots) {
    return result::UNRECOVERABLE_ERROR;
  }
  if (!bx_filters(m_opaque)[id].has_value()) {
    return result::RECOVERABLE_ERROR;
  }

  const auto status = with_stopped_controller(m_opaque, [&]() noexcept {
    return configure_bx_filter(m_opaque, bx_filters(m_opaque)[id].value(), id, false);
  });
  if (status != result::OK) {
    return status;
  }

  bx_filter_enabled(m_opaque)[id] = false;
  return result::OK;
}

expected::expected<bool, result> Bx_can::is_filter_enabled(uint8_t id) {
  if (id >= k_std_filter_slots) {
    return expected::unexpected(result::UNRECOVERABLE_ERROR);
  }

  return bx_filter_enabled(m_opaque)[id];
}

result Bx_canRx::set_priority(Bx_fifo fifo, uint8_t priority) {
  return const_cast<Bx_can&>(m_can).set_priority(fifo, priority);
}

expected::expected<uint8_t, result> Bx_canRx::get_priority(Bx_fifo fifo) {
  return const_cast<Bx_can&>(m_can).get_priority(fifo);
}

result Bx_canRx::set_interrupt(Bx_fifo fifo, bool on) {
  return const_cast<Bx_can&>(m_can).set_interrupt(fifo, on);
}

expected::expected<uint8_t, result> Bx_canRx::is_interrupt_on(Bx_fifo fifo) {
  return const_cast<Bx_can&>(m_can).is_interrupt_on(fifo);
}

result Bx_canRx::set_error_priority(uint8_t priority) {
  return const_cast<Bx_can&>(m_can).set_error_priority(priority);
}

expected::expected<uint8_t, result> Bx_canRx::get_error_priority() {
  return const_cast<Bx_can&>(m_can).get_error_priority();
}

result Bx_canRx::set_error_interrupt(bool on) {
  return const_cast<Bx_can&>(m_can).set_error_interrupt(on);
}

expected::expected<uint8_t, result> Bx_canRx::is_error_interrupt_on() {
  return const_cast<Bx_can&>(m_can).is_error_interrupt_on();
}

result Bx_canRx::set_filter(Bx_filter filter, uint8_t id) {
  return const_cast<Bx_can&>(m_can).set_filter(filter, id);
}

result Bx_canRx::enable_filter(uint8_t id) {
  return const_cast<Bx_can&>(m_can).enable_filter(id);
}

result Bx_canRx::disable_filter(uint8_t id) {
  return const_cast<Bx_can&>(m_can).disable_filter(id);
}

expected::expected<bool, result> Bx_canRx::is_filter_enabled(uint8_t id) {
  return const_cast<Bx_can&>(m_can).is_filter_enabled(id);
}

result Bx_canTx::set_error_priority(uint8_t priority) {
  return const_cast<Bx_can&>(m_can).set_error_priority(priority);
}

expected::expected<uint8_t, result> Bx_canTx::get_error_priority() {
  return const_cast<Bx_can&>(m_can).get_error_priority();
}

result Bx_canTx::set_error_interrupt(bool on) {
  return const_cast<Bx_can&>(m_can).set_error_interrupt(on);
}

expected::expected<uint8_t, result> Bx_canTx::is_error_interrupt_on() {
  return const_cast<Bx_can&>(m_can).is_error_interrupt_on();
}
}  // namespace ru::driver
