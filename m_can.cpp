#include "can/m_can.hpp"

#include <array>

#include "can_internal.hpp"

using namespace ru::driver;

namespace {
#define RU_STM32H5XX_M_CAN_ID_VALUE(name) M_canId::name,
constexpr auto k_m_can_ids =
    std::array<M_canId, static_cast<std::size_t>(M_canId::COUNT)>{
        M_CAN_LIST(RU_STM32H5XX_M_CAN_ID_VALUE)};
#undef RU_STM32H5XX_M_CAN_ID_VALUE

constexpr M_canId k_default_m_can_id = k_m_can_ids[0U];
}  // namespace

namespace ru::driver {
using namespace can_internal;

expected::expected<CanMessageTs, result> M_canRx::read(M_fifo fifo) noexcept {
  return const_cast<M_can&>(m_can).read(fifo);
}

expected::expected<CanMessageTs, result> M_canRx::try_read(M_fifo fifo) noexcept {
  return const_cast<M_can&>(m_can).try_read(fifo);
}

result M_canTx::write(const CanFrameView& message) noexcept {
  return const_cast<M_can&>(m_can).write(message);
}

result M_canTx::try_write(const CanFrameView& message) noexcept {
  return const_cast<M_can&>(m_can).try_write(message);
}

M_can::M_can(const M_canId id) noexcept : m_id(id), m_opaque(make_opaque(id)) {
}

result M_can::start() noexcept {
  return result::OK;
}

result M_can::configure(const CanControllerConfig& config) noexcept {
  return configure(k_default_m_can_id, config);
}

result M_can::configure(const M_canId id, const CanControllerConfig& config) noexcept {
  return configure_controller(make_opaque(id), config);
}

CanControllerConfig M_can::configuration() noexcept {
  return configuration(k_default_m_can_id);
}

CanControllerConfig M_can::configuration(const M_canId id) noexcept {
  return current_controller_config(make_opaque(id));
}

result M_can::init() noexcept {
  return init_controller(m_opaque);
}

result M_can::stop() noexcept {
  return stop_controller(m_opaque);
}

expected::expected<CanMessageTs, result> M_can::read(M_fifo fifo) noexcept {
  return read_fifo_message(m_opaque, fifo);
}

expected::expected<CanMessageTs, result> M_can::try_read(M_fifo fifo) noexcept {
  return read(fifo);
}

result M_can::write(const CanFrameView& message) noexcept {
  return write_controller_message(m_opaque, message);
}

result M_can::try_write(const CanFrameView& message) noexcept {
  if (!initialized(m_opaque) || !started(m_opaque)) {
    return result::RECOVERABLE_ERROR;
  }

  if (HAL_FDCAN_GetTxFifoFreeLevel(&handle(m_opaque)) == 0U) {
    return result::RECOVERABLE_ERROR;
  }

  return write(message);
}

result M_can::set_rx_callback(M_fifo fifo, void (*callback)(CanMessageTs)) {
  rx_callbacks(m_opaque)[fifo_index(fifo)] = callback;
  return refresh_notifications(m_opaque);
}

result M_can::set_txfull_callback(void (*callback)()) {
  txfull_callback(m_opaque) = callback;
  return refresh_notifications(m_opaque);
}

result M_can::set_not_matching(std::optional<M_fifo> fifo) {
  not_matching(m_opaque) = fifo;
  return with_stopped_controller(m_opaque,
                                 [&]() noexcept { return apply_global_filter(m_opaque); });
}

expected::expected<std::optional<M_fifo>, result> M_can::get_not_matching() {
  return not_matching(m_opaque);
}

result M_can::reset_timestamp() {
  return from_hal_status(HAL_FDCAN_ResetTimestampCounter(&handle(m_opaque)));
}

expected::expected<Timestamp, result> M_can::get_timestamp() {
  return static_cast<Timestamp>(HAL_FDCAN_GetTimestampCounter(&handle(m_opaque)));
}

result M_can::set_priority(M_fifo fifo, uint8_t priority) {
  rx_priorities(m_opaque)[fifo_index(fifo)] = priority;
  return refresh_notifications(m_opaque);
}

expected::expected<uint8_t, result> M_can::get_priority(M_fifo fifo) {
  return rx_priorities(m_opaque)[fifo_index(fifo)];
}

result M_can::set_interrupt(M_fifo fifo, bool on) {
  rx_interrupts(m_opaque)[fifo_index(fifo)] = on;
  return refresh_notifications(m_opaque);
}

expected::expected<uint8_t, result> M_can::is_interrupt_on(M_fifo fifo) {
  return static_cast<uint8_t>(rx_interrupts(m_opaque)[fifo_index(fifo)]);
}

result M_can::set_error_priority(uint8_t priority) {
  error_priority(m_opaque) = priority;
  return refresh_notifications(m_opaque);
}

expected::expected<uint8_t, result> M_can::get_error_priority() {
  return error_priority(m_opaque);
}

result M_can::set_error_interrupt(bool on) {
  error_interrupt(m_opaque) = on;
  return refresh_notifications(m_opaque);
}

expected::expected<uint8_t, result> M_can::is_error_interrupt_on() {
  return static_cast<uint8_t>(error_interrupt(m_opaque));
}

result M_can::set_filter(M_filter filter, uint8_t id) {
  if (id >= k_std_filter_slots) {
    return result::UNRECOVERABLE_ERROR;
  }

  const auto status = with_stopped_controller(
      m_opaque, [&]() noexcept { return configure_m_filter(m_opaque, filter, id, true); });
  if (status != result::OK) {
    return status;
  }

  m_filters(m_opaque)[id] = filter;
  m_filter_enabled(m_opaque)[id] = true;
  return result::OK;
}

result M_can::enable_filter(uint8_t id) {
  if (id >= k_std_filter_slots) {
    return result::UNRECOVERABLE_ERROR;
  }
  if (!m_filters(m_opaque)[id].has_value()) {
    return result::RECOVERABLE_ERROR;
  }

  const auto status = with_stopped_controller(m_opaque, [&]() noexcept {
    return configure_m_filter(m_opaque, m_filters(m_opaque)[id].value(), id, true);
  });
  if (status != result::OK) {
    return status;
  }

  m_filter_enabled(m_opaque)[id] = true;
  return result::OK;
}

result M_can::disable_filter(uint8_t id) {
  if (id >= k_std_filter_slots) {
    return result::UNRECOVERABLE_ERROR;
  }
  if (!m_filters(m_opaque)[id].has_value()) {
    return result::RECOVERABLE_ERROR;
  }

  const auto status = with_stopped_controller(m_opaque, [&]() noexcept {
    return configure_m_filter(m_opaque, m_filters(m_opaque)[id].value(), id, false);
  });
  if (status != result::OK) {
    return status;
  }

  m_filter_enabled(m_opaque)[id] = false;
  return result::OK;
}

expected::expected<bool, result> M_can::is_filter_enabled(uint8_t id) {
  if (id >= k_std_filter_slots) {
    return expected::unexpected(result::UNRECOVERABLE_ERROR);
  }

  return m_filter_enabled(m_opaque)[id];
}

result M_canRx::set_rx_callback(M_fifo fifo, void (*callback)(CanMessageTs)) {
  return const_cast<M_can&>(m_can).set_rx_callback(fifo, callback);
}

result M_canRx::set_not_matching(std::optional<M_fifo> fifo) {
  return const_cast<M_can&>(m_can).set_not_matching(fifo);
}

expected::expected<std::optional<M_fifo>, result> M_canRx::get_not_matching() {
  return const_cast<M_can&>(m_can).get_not_matching();
}

result M_canRx::reset_timestamp() {
  return const_cast<M_can&>(m_can).reset_timestamp();
}

expected::expected<Timestamp, result> M_canRx::get_timestamp() {
  return const_cast<M_can&>(m_can).get_timestamp();
}

result M_canRx::set_priority(M_fifo fifo, uint8_t priority) {
  return const_cast<M_can&>(m_can).set_priority(fifo, priority);
}

expected::expected<uint8_t, result> M_canRx::get_priority(M_fifo fifo) {
  return const_cast<M_can&>(m_can).get_priority(fifo);
}

result M_canRx::set_interrupt(M_fifo fifo, bool on) {
  return const_cast<M_can&>(m_can).set_interrupt(fifo, on);
}

expected::expected<uint8_t, result> M_canRx::is_interrupt_on(M_fifo fifo) {
  return const_cast<M_can&>(m_can).is_interrupt_on(fifo);
}

result M_canRx::set_error_priority(uint8_t priority) {
  return const_cast<M_can&>(m_can).set_error_priority(priority);
}

expected::expected<uint8_t, result> M_canRx::get_error_priority() {
  return const_cast<M_can&>(m_can).get_error_priority();
}

result M_canRx::set_error_interrupt(bool on) {
  return const_cast<M_can&>(m_can).set_error_interrupt(on);
}

expected::expected<uint8_t, result> M_canRx::is_error_interrupt_on() {
  return const_cast<M_can&>(m_can).is_error_interrupt_on();
}

result M_canRx::set_filter(M_filter filter, uint8_t id) {
  return const_cast<M_can&>(m_can).set_filter(filter, id);
}

result M_canRx::enable_filter(uint8_t id) {
  return const_cast<M_can&>(m_can).enable_filter(id);
}

result M_canRx::disable_filter(uint8_t id) {
  return const_cast<M_can&>(m_can).disable_filter(id);
}

expected::expected<bool, result> M_canRx::is_filter_enabled(uint8_t id) {
  return const_cast<M_can&>(m_can).is_filter_enabled(id);
}

result M_canTx::set_txfull_callback(void (*callback)()) {
  return const_cast<M_can&>(m_can).set_txfull_callback(callback);
}

result M_canTx::reset_timestamp() {
  return const_cast<M_can&>(m_can).reset_timestamp();
}

expected::expected<Timestamp, result> M_canTx::get_timestamp() {
  return const_cast<M_can&>(m_can).get_timestamp();
}

result M_canTx::set_error_priority(uint8_t priority) {
  return const_cast<M_can&>(m_can).set_error_priority(priority);
}

expected::expected<uint8_t, result> M_canTx::get_error_priority() {
  return const_cast<M_can&>(m_can).get_error_priority();
}

result M_canTx::set_error_interrupt(bool on) {
  return const_cast<M_can&>(m_can).set_error_interrupt(on);
}

expected::expected<uint8_t, result> M_canTx::is_error_interrupt_on() {
  return const_cast<M_can&>(m_can).is_error_interrupt_on();
}
}  // namespace ru::driver
