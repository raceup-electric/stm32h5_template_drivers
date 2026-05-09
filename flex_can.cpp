#include "can/flex_can.hpp"

#include "can_internal.hpp"

using namespace ru::driver;

namespace ru::driver {
using namespace can_internal;

Flex_can::Flex_can(const Flex_canId id) noexcept : m_id(id), m_opaque(make_opaque(id)) {
}

result Flex_can::start() noexcept {
  return result::UNRECOVERABLE_ERROR;
}

result Flex_can::init() noexcept {
  return result::UNRECOVERABLE_ERROR;
}

result Flex_can::stop() noexcept {
  return result::UNRECOVERABLE_ERROR;
}

expected::expected<CanMessageTs, result> Flex_can::read() noexcept {
  return unrecoverable_expected<CanMessageTs>();
}

expected::expected<CanMessageTs, result> Flex_can::read(uint8_t mb) noexcept {
  (void)mb;
  return unrecoverable_expected<CanMessageTs>();
}

expected::expected<CanMessageTs, result> Flex_can::try_read() noexcept {
  return unrecoverable_expected<CanMessageTs>();
}

expected::expected<CanMessageTs, result> Flex_can::try_read(uint8_t mb) noexcept {
  (void)mb;
  return unrecoverable_expected<CanMessageTs>();
}

result Flex_can::write(const CanFrameView& message) noexcept {
  (void)message;
  return unrecoverable_result();
}

result Flex_can::try_write(const CanFrameView& message) noexcept {
  (void)message;
  return unrecoverable_result();
}

result Flex_can::set_rx_callback(void (*callback)(CanMessageTs)) {
  (void)callback;
  return unrecoverable_result();
}

result Flex_can::enable_rx_interrupt(uint8_t id) {
  (void)id;
  return unrecoverable_result();
}

result Flex_can::disable_rx_interrupt(uint8_t id) {
  (void)id;
  return unrecoverable_result();
}

expected::expected<Timestamp, result> Flex_can::get_timestamp() {
  return unrecoverable_expected<Timestamp>();
}

result Flex_can::set_rx_priority(uint8_t priority) {
  (void)priority;
  return unrecoverable_result();
}

expected::expected<uint8_t, result> Flex_can::get_rx_priority() {
  return unrecoverable_expected<uint8_t>();
}

result Flex_can::set_rx_interrupt(bool on) {
  (void)on;
  return unrecoverable_result();
}

expected::expected<uint8_t, result> Flex_can::is_rx_interrupt_on() {
  return unrecoverable_expected<uint8_t>();
}

result Flex_can::set_error_priority(uint8_t priority) {
  (void)priority;
  return unrecoverable_result();
}

expected::expected<uint8_t, result> Flex_can::get_error_priority() {
  return unrecoverable_expected<uint8_t>();
}

result Flex_can::set_error_interrupt(bool on) {
  (void)on;
  return unrecoverable_result();
}

expected::expected<uint8_t, result> Flex_can::is_error_interrupt_on() {
  return unrecoverable_expected<uint8_t>();
}

result Flex_can::set_filter(Flex_filter filter, uint8_t id) {
  (void)filter;
  (void)id;
  return unrecoverable_result();
}

result Flex_can::enable_filter(uint8_t id) {
  (void)id;
  return unrecoverable_result();
}

result Flex_can::disable_filter(uint8_t id) {
  (void)id;
  return unrecoverable_result();
}

expected::expected<bool, result> Flex_can::is_filter_enabled(uint8_t id) {
  (void)id;
  return unrecoverable_expected<bool>();
}

result Flex_can::set_fifo_mask(uint16_t* ids, uint8_t len) {
  (void)ids;
  (void)len;
  return unrecoverable_result();
}

expected::expected<CanMessageTs, result> Flex_canRx::read() noexcept {
  return const_cast<Flex_can&>(m_can).read();
}

expected::expected<CanMessageTs, result> Flex_canRx::read(uint8_t mb) noexcept {
  return const_cast<Flex_can&>(m_can).read(mb);
}

expected::expected<CanMessageTs, result> Flex_canRx::try_read() noexcept {
  return const_cast<Flex_can&>(m_can).try_read();
}

expected::expected<CanMessageTs, result> Flex_canRx::try_read(uint8_t mb) noexcept {
  return const_cast<Flex_can&>(m_can).try_read(mb);
}

result Flex_canRx::set_rx_callback(void (*callback)(CanMessageTs)) {
  return const_cast<Flex_can&>(m_can).set_rx_callback(callback);
}

result Flex_canRx::enable_rx_interrupt(uint8_t id) {
  return const_cast<Flex_can&>(m_can).enable_rx_interrupt(id);
}

result Flex_canRx::disable_rx_interrupt(uint8_t id) {
  return const_cast<Flex_can&>(m_can).disable_rx_interrupt(id);
}

expected::expected<Timestamp, result> Flex_canRx::get_timestamp() {
  return const_cast<Flex_can&>(m_can).get_timestamp();
}

result Flex_canRx::set_rx_priority(uint8_t priority) {
  return const_cast<Flex_can&>(m_can).set_rx_priority(priority);
}

expected::expected<uint8_t, result> Flex_canRx::get_rx_priority() {
  return const_cast<Flex_can&>(m_can).get_rx_priority();
}

result Flex_canRx::set_rx_interrupt(bool on) {
  return const_cast<Flex_can&>(m_can).set_rx_interrupt(on);
}

expected::expected<uint8_t, result> Flex_canRx::is_rx_interrupt_on() {
  return const_cast<Flex_can&>(m_can).is_rx_interrupt_on();
}

result Flex_canRx::set_error_priority(uint8_t priority) {
  return const_cast<Flex_can&>(m_can).set_error_priority(priority);
}

expected::expected<uint8_t, result> Flex_canRx::get_error_priority() {
  return const_cast<Flex_can&>(m_can).get_error_priority();
}

result Flex_canRx::set_error_interrupt(bool on) {
  return const_cast<Flex_can&>(m_can).set_error_interrupt(on);
}

expected::expected<uint8_t, result> Flex_canRx::is_error_interrupt_on() {
  return const_cast<Flex_can&>(m_can).is_error_interrupt_on();
}

result Flex_canRx::set_filter(Flex_filter filter) {
  return const_cast<Flex_can&>(m_can).set_filter(filter, filter.MB);
}

result Flex_canRx::enable_filter(uint8_t id) {
  return const_cast<Flex_can&>(m_can).enable_filter(id);
}

result Flex_canRx::disable_filter(uint8_t id) {
  return const_cast<Flex_can&>(m_can).disable_filter(id);
}

expected::expected<bool, result> Flex_canRx::is_filter_enabled(uint8_t id) {
  return const_cast<Flex_can&>(m_can).is_filter_enabled(id);
}

result Flex_canTx::write(const CanFrameView& message) noexcept {
  return const_cast<Flex_can&>(m_can).write(message);
}

result Flex_canTx::try_write(const CanFrameView& message) noexcept {
  return const_cast<Flex_can&>(m_can).try_write(message);
}

expected::expected<Timestamp, result> Flex_canTx::get_timestamp() {
  return const_cast<Flex_can&>(m_can).get_timestamp();
}

result Flex_canTx::set_error_priority(uint8_t priority) {
  return const_cast<Flex_can&>(m_can).set_error_priority(priority);
}

expected::expected<uint8_t, result> Flex_canTx::get_error_priority() {
  return const_cast<Flex_can&>(m_can).get_error_priority();
}

result Flex_canTx::set_error_interrupt(bool on) {
  return const_cast<Flex_can&>(m_can).set_error_interrupt(on);
}

expected::expected<uint8_t, result> Flex_canTx::is_error_interrupt_on() {
  return const_cast<Flex_can&>(m_can).is_error_interrupt_on();
}

result Flex_canTx::set_filter(Flex_filter filter) {
  return const_cast<Flex_can&>(m_can).set_filter(filter, filter.MB);
}

result Flex_canTx::enable_filter(uint8_t id) {
  return const_cast<Flex_can&>(m_can).enable_filter(id);
}

result Flex_canTx::disable_filter(uint8_t id) {
  return const_cast<Flex_can&>(m_can).disable_filter(id);
}

expected::expected<bool, result> Flex_canTx::is_filter_enabled(uint8_t id) {
  return const_cast<Flex_can&>(m_can).is_filter_enabled(id);
}
}  // namespace ru::driver
