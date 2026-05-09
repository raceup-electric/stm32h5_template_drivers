#include "can/multi_can.hpp"

#include "can_internal.hpp"

using namespace ru::driver;

namespace ru::driver {
using namespace can_internal;

expected::expected<CanMessageTs, result> CanRx::read(uint8_t buffer) noexcept {
  (void)buffer;
  return unrecoverable_expected<CanMessageTs>();
}

expected::expected<CanMessageTs, result> CanRx::read(Multi_fifo fifo) noexcept {
  (void)fifo;
  return unrecoverable_expected<CanMessageTs>();
}

expected::expected<CanMessageTs, result> CanRx::try_read(uint8_t buffer) noexcept {
  (void)buffer;
  return unrecoverable_expected<CanMessageTs>();
}

expected::expected<CanMessageTs, result> CanRx::try_read(Multi_fifo fifo) noexcept {
  (void)fifo;
  return unrecoverable_expected<CanMessageTs>();
}

result CanTx::write(const CanFrameView& message) noexcept {
  (void)message;
  return unrecoverable_result();
}

result CanTx::try_write(const CanFrameView& message) noexcept {
  (void)message;
  return unrecoverable_result();
}

Can::Can(const Multi_canId id) noexcept : m_id(id) {
}

result Can::start() noexcept {
  return result::UNRECOVERABLE_ERROR;
}

result Can::init() noexcept {
  return result::UNRECOVERABLE_ERROR;
}

result Can::stop() noexcept {
  return result::UNRECOVERABLE_ERROR;
}

expected::expected<CanMessageTs, result> Can::read(uint8_t buffer) noexcept {
  (void)buffer;
  return unrecoverable_expected<CanMessageTs>();
}

expected::expected<CanMessageTs, result> Can::read(Multi_fifo fifo) noexcept {
  (void)fifo;
  return unrecoverable_expected<CanMessageTs>();
}

expected::expected<CanMessageTs, result> Can::try_read(uint8_t buffer) noexcept {
  (void)buffer;
  return unrecoverable_expected<CanMessageTs>();
}

expected::expected<CanMessageTs, result> Can::try_read(Multi_fifo fifo) noexcept {
  (void)fifo;
  return unrecoverable_expected<CanMessageTs>();
}

result Can::write(const CanFrameView& message) noexcept {
  (void)message;
  return unrecoverable_result();
}

result Can::try_write(const CanFrameView& message) noexcept {
  (void)message;
  return unrecoverable_result();
}

result Can::set_rx_callback(Multi_fifo fifo, void (*callback)(CanMessageTs)) {
  (void)fifo;
  (void)callback;
  return unrecoverable_result();
}

result Can::set_rx_callback(uint8_t buffer, void (*callback)(CanMessageTs)) {
  (void)buffer;
  (void)callback;
  return unrecoverable_result();
}

result Can::enable_rx_interrupt(Multi_fifo fifo) {
  (void)fifo;
  return unrecoverable_result();
}

result Can::enable_rx_interrupt(uint8_t buffer) {
  (void)buffer;
  return unrecoverable_result();
}

result Can::disable_rx_interrupt(Multi_fifo fifo) {
  (void)fifo;
  return unrecoverable_result();
}

result Can::disable_rx_interrupt(uint8_t buffer) {
  (void)buffer;
  return unrecoverable_result();
}

expected::expected<Timestamp, result> Can::get_timestamp() {
  return unrecoverable_expected<Timestamp>();
}

result Can::set_rx_priority(Multi_fifo fifo, uint8_t priority) {
  (void)fifo;
  (void)priority;
  return unrecoverable_result();
}

result Can::set_rx_priority(uint8_t priority) {
  (void)priority;
  return unrecoverable_result();
}

expected::expected<uint8_t, result> Can::get_rx_priority() {
  return unrecoverable_expected<uint8_t>();
}

expected::expected<uint8_t, result> Can::get_rx_priority(Multi_fifo fifo) {
  (void)fifo;
  return unrecoverable_expected<uint8_t>();
}

result Can::set_rx_interrupt(Multi_fifo fifo, bool on) {
  (void)fifo;
  (void)on;
  return unrecoverable_result();
}

result Can::set_rx_interrupt(bool on) {
  (void)on;
  return unrecoverable_result();
}

expected::expected<uint8_t, result> Can::is_rx_interrupt_on(Multi_fifo fifo) {
  (void)fifo;
  return unrecoverable_expected<uint8_t>();
}

expected::expected<uint8_t, result> Can::is_rx_interrupt_on() {
  return unrecoverable_expected<uint8_t>();
}

result Can::set_error_priority(uint8_t priority) {
  (void)priority;
  return unrecoverable_result();
}

expected::expected<uint8_t, result> Can::get_error_priority() {
  return unrecoverable_expected<uint8_t>();
}

result Can::set_error_interrupt(bool on) {
  (void)on;
  return unrecoverable_result();
}

expected::expected<uint8_t, result> Can::is_error_interrupt_on() {
  return unrecoverable_expected<uint8_t>();
}

result Can::set_filter(Multi_filter filter, uint8_t id) {
  (void)filter;
  (void)id;
  return unrecoverable_result();
}

result Can::enable_filter(uint8_t id) {
  (void)id;
  return unrecoverable_result();
}

result Can::disable_filter(uint8_t id) {
  (void)id;
  return unrecoverable_result();
}

expected::expected<bool, result> Can::is_filter_enabled(uint8_t id) {
  (void)id;
  return unrecoverable_expected<bool>();
}

result CanRx::set_rx_callback(Multi_fifo fifo, void (*callback)(CanMessageTs)) {
  return const_cast<Can&>(m_can).set_rx_callback(fifo, callback);
}

result CanRx::set_rx_callback(uint8_t buffer, void (*callback)(CanMessageTs)) {
  return const_cast<Can&>(m_can).set_rx_callback(buffer, callback);
}

result CanRx::enable_rx_interrupt(Multi_fifo fifo) {
  return const_cast<Can&>(m_can).enable_rx_interrupt(fifo);
}

result CanRx::enable_rx_interrupt(uint8_t buffer) {
  return const_cast<Can&>(m_can).enable_rx_interrupt(buffer);
}

result CanRx::disable_rx_interrupt(Multi_fifo fifo) {
  return const_cast<Can&>(m_can).disable_rx_interrupt(fifo);
}

result CanRx::disable_rx_interrupt(uint8_t buffer) {
  return const_cast<Can&>(m_can).disable_rx_interrupt(buffer);
}

expected::expected<Timestamp, result> CanRx::get_timestamp() {
  return const_cast<Can&>(m_can).get_timestamp();
}

result CanRx::set_rx_priority(Multi_fifo fifo, uint8_t priority) {
  return const_cast<Can&>(m_can).set_rx_priority(fifo, priority);
}

result CanRx::set_rx_priority(uint8_t priority) {
  return const_cast<Can&>(m_can).set_rx_priority(priority);
}

expected::expected<uint8_t, result> CanRx::get_rx_priority() {
  return const_cast<Can&>(m_can).get_rx_priority();
}

expected::expected<uint8_t, result> CanRx::get_rx_priority(Multi_fifo fifo) {
  return const_cast<Can&>(m_can).get_rx_priority(fifo);
}

result CanRx::set_rx_interrupt(Multi_fifo fifo, bool on) {
  return const_cast<Can&>(m_can).set_rx_interrupt(fifo, on);
}

result CanRx::set_rx_interrupt(bool on) {
  return const_cast<Can&>(m_can).set_rx_interrupt(on);
}

expected::expected<uint8_t, result> CanRx::is_rx_interrupt_on(Multi_fifo fifo) {
  return const_cast<Can&>(m_can).is_rx_interrupt_on(fifo);
}

expected::expected<uint8_t, result> CanRx::is_rx_interrupt_on() {
  return const_cast<Can&>(m_can).is_rx_interrupt_on();
}

result CanRx::set_error_priority(uint8_t priority) {
  return const_cast<Can&>(m_can).set_error_priority(priority);
}

expected::expected<uint8_t, result> CanRx::get_error_priority() {
  return const_cast<Can&>(m_can).get_error_priority();
}

result CanRx::set_error_interrupt(bool on) {
  return const_cast<Can&>(m_can).set_error_interrupt(on);
}

expected::expected<uint8_t, result> CanRx::is_error_interrupt_on() {
  return const_cast<Can&>(m_can).is_error_interrupt_on();
}

result CanRx::set_filter(Multi_filter filter, uint8_t id) {
  return const_cast<Can&>(m_can).set_filter(filter, id);
}

result CanRx::enable_filter(uint8_t id) {
  return const_cast<Can&>(m_can).enable_filter(id);
}

result CanRx::disable_filter(uint8_t id) {
  return const_cast<Can&>(m_can).disable_filter(id);
}

expected::expected<bool, result> CanRx::is_filter_enabled(uint8_t id) {
  return const_cast<Can&>(m_can).is_filter_enabled(id);
}

expected::expected<Timestamp, result> CanTx::get_timestamp() {
  return const_cast<Can&>(m_can).get_timestamp();
}

result CanTx::set_error_priority(uint8_t priority) {
  return const_cast<Can&>(m_can).set_error_priority(priority);
}

expected::expected<uint8_t, result> CanTx::get_error_priority() {
  return const_cast<Can&>(m_can).get_error_priority();
}

result CanTx::set_error_interrupt(bool on) {
  return const_cast<Can&>(m_can).set_error_interrupt(on);
}

expected::expected<uint8_t, result> CanTx::is_error_interrupt_on() {
  return const_cast<Can&>(m_can).is_error_interrupt_on();
}
}  // namespace ru::driver
