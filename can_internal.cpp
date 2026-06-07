#include "can_internal.hpp"

#include <algorithm>
#include <cstring>
#include <type_traits>

using namespace ru::driver;

namespace ru::driver::can_internal {
namespace {
constexpr std::size_t k_can_classic_max_payload = 8U;
constexpr std::size_t k_can_fd_max_payload = 64U;

enum class CanIdFormat : uint8_t { Standard, Extended };
enum class CanFrameFormat : uint8_t { Classic, Fd, FdBrs };

struct CanControllerConfig {
  uint8_t standard_filter_count{static_cast<uint8_t>(k_std_filter_slots)};
  uint8_t extended_filter_count{static_cast<uint8_t>(k_ext_filter_slots)};
  CanFrameFormat max_frame_format{CanFrameFormat::Classic};
};

constexpr bool can_id_value_is_valid(const CanIdFormat format,
                                     const uint32_t value) noexcept {
  return format == CanIdFormat::Extended ? value <= 0x1FFFFFFFU : value <= 0x7FFU;
}
}  // namespace
namespace {
constexpr uint32_t hw_fifo(const M_fifo fifo) noexcept {
  return fifo == M_fifo::FIFO1 ? FDCAN_RX_FIFO1 : FDCAN_RX_FIFO0;
}

constexpr uint32_t global_filter_target(const std::optional<M_fifo>& fifo) noexcept {
  if (!fifo.has_value()) {
    return FDCAN_REJECT;
  }

  return fifo.value() == M_fifo::FIFO1 ? FDCAN_ACCEPT_IN_RX_FIFO1 : FDCAN_ACCEPT_IN_RX_FIFO0;
}

constexpr uint32_t fifo_filter_target(const M_fifo fifo) noexcept {
  return fifo == M_fifo::FIFO1 ? FDCAN_FILTER_TO_RXFIFO1 : FDCAN_FILTER_TO_RXFIFO0;
}

constexpr uint32_t fdcan_id_type(const CanIdFormat format) noexcept {
  return format == CanIdFormat::Extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
}

constexpr uint32_t fdcan_controller_frame_format(const CanFrameFormat format) noexcept {
  switch (format) {
    case CanFrameFormat::FdBrs:
      return FDCAN_FRAME_FD_BRS;
    case CanFrameFormat::Fd:
      return FDCAN_FRAME_FD_NO_BRS;
    default:
      return FDCAN_FRAME_CLASSIC;
  }
}

constexpr uint32_t fdcan_tx_frame_format(const CanFrameFormat format) noexcept {
  return format == CanFrameFormat::Classic ? FDCAN_CLASSIC_CAN : FDCAN_FD_CAN;
}

constexpr uint32_t fdcan_bitrate_switch(const CanFrameFormat format) noexcept {
  return format == CanFrameFormat::FdBrs ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
}

constexpr CanIdFormat can_id_format_from_header(
    const FDCAN_RxHeaderTypeDef& header) noexcept {
  return header.IdType == FDCAN_EXTENDED_ID ? CanIdFormat::Extended
                                           : CanIdFormat::Standard;
}

constexpr CanFrameFormat can_frame_format_from_header(
    const FDCAN_RxHeaderTypeDef& header) noexcept {
  if (header.FDFormat != FDCAN_FD_CAN) {
    return CanFrameFormat::Classic;
  }

  return header.BitRateSwitch == FDCAN_BRS_ON ? CanFrameFormat::FdBrs
                                              : CanFrameFormat::Fd;
}

constexpr bool controller_config_is_valid(
    const CanControllerConfig& controller_config) noexcept {
  return controller_config.standard_filter_count <= k_std_filter_slots &&
         controller_config.extended_filter_count <= k_ext_filter_slots;
}

constexpr bool filter_slot_is_valid(const CanIdFormat format, const uint8_t id,
                                    const CanControllerConfig& controller_config) noexcept {
  return format == CanIdFormat::Extended
             ? id < controller_config.extended_filter_count
             : id < controller_config.standard_filter_count;
}

constexpr bool filter_value_is_valid(const CanIdFormat format,
                                     const uint32_t value) noexcept {
  return can_id_value_is_valid(format, value);
}

constexpr bool filter_range_is_valid(const CanIdFormat format, const uint32_t from,
                                     const uint32_t to) noexcept {
  return from <= to && filter_value_is_valid(format, from) &&
         filter_value_is_valid(format, to);
}

constexpr uint32_t normalize_bx16_value(const uint16_t value) noexcept {
  return value <= 0x7FFU ? value : ((value >> 5U) & 0x7FFU);
}

CanMessageTs make_message_ts(const FDCAN_RxHeaderTypeDef& header,
                            const uint8_t* const payload) noexcept {
  const auto len = std::min(fdcan_length_from_dlc(header.DataLength),
                            static_cast<uint8_t>(k_can_classic_max_payload));
  const auto message_len = static_cast<uint8_t>(len & 0x0FU);
  CanMessage message{};
  message.id = header.Identifier & 0x7FFU;
  message.len = message_len;
  std::memcpy(message.bytes, payload, message_len);

  const auto filter =
      header.IsFilterMatchingFrame == 0U
          ? std::optional<uint8_t>{static_cast<uint8_t>(header.FilterIndex)}
          : std::optional<uint8_t>{};
  return CanMessageTs{message, filter, static_cast<Timestamp>(header.RxTimestamp)};
}

struct CanControllerState {
  FDCAN_HandleTypeDef handle{};
  std::array<uint8_t, 2> rx_priorities{5U, 5U};
  std::array<bool, 2> rx_interrupts{false, false};
  uint8_t error_priority{5U};
  bool error_interrupt{false};
  CanControllerConfig controller_config{};
  std::optional<M_fifo> not_matching{};
  std::array<std::optional<M_filter>, k_std_filter_slots> m_filters{};
  std::array<bool, k_std_filter_slots> m_filter_enabled{};
  std::array<std::optional<Bx_filter>, k_std_filter_slots> bx_filters{};
  std::array<bool, k_std_filter_slots> bx_filter_enabled{};
  std::array<void (*)(), 2> rx_callbacks{nullptr, nullptr};
  void (*txfull_callback)(){nullptr};
};

CanControllerState g_fdcan1_state{};
#if defined(FDCAN2)
CanControllerState g_fdcan2_state{};
#endif
CanControllerState g_invalid_state{};

CanControllerState& state(const opaque_can& config) noexcept {
  if (config.m_key == can_controller_key::Fdcan1) {
    return g_fdcan1_state;
  }
#if defined(FDCAN2)
  if (config.m_key == can_controller_key::Fdcan2) {
    return g_fdcan2_state;
  }
#endif

  return g_invalid_state;
}

inline bool instance_matches_key(const opaque_can& config,
                                 FDCAN_GlobalTypeDef* const p_instance) noexcept {
  return make_opaque(p_instance).m_key == config.m_key;
}

constexpr IRQn_Type fdcan_it0_irq(const opaque_can& config) noexcept {
#if defined(FDCAN2)
  if (config.m_key == can_controller_key::Fdcan2) {
    return FDCAN2_IT0_IRQn;
  }
#else
  (void)config;
#endif

  return FDCAN1_IT0_IRQn;
}

constexpr IRQn_Type fdcan_it1_irq(const opaque_can& config) noexcept {
#if defined(FDCAN2)
  if (config.m_key == can_controller_key::Fdcan2) {
    return FDCAN2_IT1_IRQn;
  }
#else
  (void)config;
#endif

  return FDCAN1_IT1_IRQn;
}

void update_irq_priorities(const opaque_can& config) noexcept {
  const auto& fifo_priorities = rx_priorities(config);
  const auto line0_priority = fifo_priorities[0];
  const auto line1_priority = std::min(fifo_priorities[1], error_priority(config));

  HAL_NVIC_SetPriority(fdcan_it0_irq(config), line0_priority, 0U);
  HAL_NVIC_SetPriority(fdcan_it1_irq(config), line1_priority, 0U);
}

result do_apply_global_filter(const opaque_can& config) noexcept {
  return from_hal_status(HAL_FDCAN_ConfigGlobalFilter(
      &handle(config), global_filter_target(not_matching(config)),
      global_filter_target(not_matching(config)), FDCAN_REJECT_REMOTE,
      FDCAN_REJECT_REMOTE));
}

bool translate_m_filter(const M_filter& filter, FDCAN_FilterTypeDef& hw_filter) noexcept {
  hw_filter.FilterConfig = fifo_filter_target(filter.fifo);
  hw_filter.IdType = FDCAN_STANDARD_ID;

  return std::visit(
      [&](const auto& config) noexcept -> bool {
        using config_type = std::decay_t<decltype(config)>;

        if constexpr (std::is_same_v<config_type, M_Mask>) {
          if (!filter_value_is_valid(CanIdFormat::Standard, config.id) ||
              !filter_value_is_valid(CanIdFormat::Standard, config.mask)) {
            return false;
          }
          hw_filter.FilterType = FDCAN_FILTER_MASK;
          hw_filter.FilterID1 = config.id;
          hw_filter.FilterID2 = config.mask;
          return true;
        } else if constexpr (std::is_same_v<config_type, M_Dual>) {
          if (!filter_value_is_valid(CanIdFormat::Standard, config.id1) ||
              !filter_value_is_valid(CanIdFormat::Standard, config.id2)) {
            return false;
          }
          hw_filter.FilterType = FDCAN_FILTER_DUAL;
          hw_filter.FilterID1 = config.id1;
          hw_filter.FilterID2 = config.id2;
          return true;
        } else if constexpr (std::is_same_v<config_type, M_Range>) {
          if (!filter_range_is_valid(CanIdFormat::Standard, config.from, config.to)) {
            return false;
          }
          hw_filter.FilterType = FDCAN_FILTER_RANGE;
          hw_filter.FilterID1 = config.from;
          hw_filter.FilterID2 = config.to;
          return true;
        } else {
          return false;
        }
      },
      filter.config);
}

bool translate_bx_filter(const Bx_filter& filter, FDCAN_FilterTypeDef& hw_filter) noexcept {
  hw_filter.FilterConfig = fifo_filter_target(to_m_fifo(filter.fifo));

  return std::visit(
      [&](const auto& config) noexcept -> bool {
        using config_type = std::decay_t<decltype(config)>;

        if constexpr (std::is_same_v<config_type, Bx_Mask32>) {
          if (!filter_value_is_valid(CanIdFormat::Standard, config.id) ||
              !filter_value_is_valid(CanIdFormat::Standard, config.mask)) {
            return false;
          }
          hw_filter.IdType = FDCAN_STANDARD_ID;
          hw_filter.FilterType = FDCAN_FILTER_MASK;
          hw_filter.FilterID1 = config.id;
          hw_filter.FilterID2 = config.mask;
          return true;
        } else if constexpr (std::is_same_v<config_type, Bx_List32>) {
          if (!filter_value_is_valid(CanIdFormat::Standard, config.id1) ||
              !filter_value_is_valid(CanIdFormat::Standard, config.id2)) {
            return false;
          }
          hw_filter.IdType = FDCAN_STANDARD_ID;
          hw_filter.FilterType = FDCAN_FILTER_DUAL;
          hw_filter.FilterID1 = config.id1;
          hw_filter.FilterID2 = config.id2;
          return true;
        } else if constexpr (std::is_same_v<config_type, Bx_Mask16>) {
          hw_filter.IdType = FDCAN_STANDARD_ID;
          hw_filter.FilterType = FDCAN_FILTER_MASK;
          hw_filter.FilterID1 = normalize_bx16_value(config.id1);
          hw_filter.FilterID2 = normalize_bx16_value(config.mask1);
          return true;
        } else if constexpr (std::is_same_v<config_type, Bx_List16>) {
          hw_filter.IdType = FDCAN_STANDARD_ID;
          hw_filter.FilterType = FDCAN_FILTER_DUAL;
          hw_filter.FilterID1 = normalize_bx16_value(config.id1);
          hw_filter.FilterID2 = normalize_bx16_value(config.id2);
          return true;
        } else {
          return false;
        }
      },
      filter.config);
}

CanIdFormat m_filter_format(const M_filter& filter) noexcept {
  (void)filter;
  return CanIdFormat::Standard;
}

CanIdFormat bx_filter_format(const Bx_filter& filter) noexcept {
  (void)filter;
  return CanIdFormat::Standard;
}
}  // namespace

FDCAN_HandleTypeDef& handle(const opaque_can& config) noexcept {
  return state(config).handle;
}

bool initialized(const opaque_can& config) noexcept {
  return handle(config).Instance != nullptr;
}

bool started(const opaque_can& config) noexcept {
  const auto* const p_instance = handle(config).Instance;
  return p_instance != nullptr && (p_instance->CCCR & FDCAN_CCCR_INIT) == 0U;
}

std::array<uint8_t, 2>& rx_priorities(const opaque_can& config) noexcept {
  return state(config).rx_priorities;
}

std::array<bool, 2>& rx_interrupts(const opaque_can& config) noexcept {
  return state(config).rx_interrupts;
}

uint8_t& error_priority(const opaque_can& config) noexcept {
  return state(config).error_priority;
}

bool& error_interrupt(const opaque_can& config) noexcept {
  return state(config).error_interrupt;
}

CanControllerConfig& controller_config(const opaque_can& config) noexcept {
  return state(config).controller_config;
}

std::optional<M_fifo>& not_matching(const opaque_can& config) noexcept {
  return state(config).not_matching;
}

std::array<std::optional<M_filter>, k_std_filter_slots>& m_filters(
    const opaque_can& config) noexcept {
  return state(config).m_filters;
}

std::array<bool, k_std_filter_slots>& m_filter_enabled(const opaque_can& config) noexcept {
  return state(config).m_filter_enabled;
}

std::array<std::optional<Bx_filter>, k_std_filter_slots>& bx_filters(
    const opaque_can& config) noexcept {
  return state(config).bx_filters;
}

std::array<bool, k_std_filter_slots>& bx_filter_enabled(const opaque_can& config) noexcept {
  return state(config).bx_filter_enabled;
}

std::array<void (*)(), 2>& rx_callbacks(const opaque_can& config) noexcept {
  return state(config).rx_callbacks;
}

void (*&txfull_callback(const opaque_can& config))() {
  return state(config).txfull_callback;
}
result apply_global_filter(const opaque_can& config) noexcept {
  return do_apply_global_filter(config);
}

result refresh_notifications(const opaque_can& config) noexcept {
  update_irq_priorities(config);

  if (!initialized(config) || !started(config)) {
    return result::OK;
  }

  auto& hw_handle = handle(config);
  if (HAL_FDCAN_DeactivateNotification(
          &hw_handle,
          FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE |
              FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_WARNING | FDCAN_IT_ERROR_PASSIVE |
              FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR |
              FDCAN_IT_RAM_ACCESS_FAILURE | FDCAN_IT_ERROR_LOGGING_OVERFLOW |
              FDCAN_IT_RESERVED_ADDRESS_ACCESS | FDCAN_IT_TX_COMPLETE) != HAL_OK) {
    return result::RECOVERABLE_ERROR;
  }

  if (HAL_FDCAN_ActivateNotification(&hw_handle, FDCAN_IT_BUS_OFF, 0U) != HAL_OK) {
    return result::RECOVERABLE_ERROR;
  }

  if (rx_interrupts(config)[0] || rx_callbacks(config)[0] != nullptr) {
    if (HAL_FDCAN_ActivateNotification(&hw_handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) !=
        HAL_OK) {
      return result::RECOVERABLE_ERROR;
    }
  }

  if (rx_interrupts(config)[1] || rx_callbacks(config)[1] != nullptr) {
    if (HAL_FDCAN_ActivateNotification(&hw_handle, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0U) !=
        HAL_OK) {
      return result::RECOVERABLE_ERROR;
    }
  }

  if (error_interrupt(config)) {
    if (HAL_FDCAN_ActivateNotification(
            &hw_handle,
            FDCAN_IT_ERROR_WARNING | FDCAN_IT_ERROR_PASSIVE |
                FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR |
                FDCAN_IT_RAM_ACCESS_FAILURE | FDCAN_IT_ERROR_LOGGING_OVERFLOW |
                FDCAN_IT_RESERVED_ADDRESS_ACCESS,
            0U) != HAL_OK) {
      return result::RECOVERABLE_ERROR;
    }
  }

  if (txfull_callback(config) != nullptr) {
    if (HAL_FDCAN_ActivateNotification(&hw_handle, FDCAN_IT_TX_COMPLETE,
                                       FDCAN_TX_BUFFER0) != HAL_OK) {
      return result::RECOVERABLE_ERROR;
    }
  }

  HAL_NVIC_EnableIRQ(fdcan_it0_irq(config));
  HAL_NVIC_EnableIRQ(fdcan_it1_irq(config));
  return result::OK;
}

result init_controller(const opaque_can& config,
                       const stm32h5xx::cfg::can_config& init_config) noexcept {
  if (config.m_key == can_controller_key::Invalid || init_config.instance() == nullptr ||
      init_config.port_rx() == nullptr || init_config.port_tx() == nullptr ||
      !instance_matches_key(config, init_config.instance())) {
    return result::UNRECOVERABLE_ERROR;
  }
  if (!controller_config_is_valid(controller_config(config))) {
    return result::UNRECOVERABLE_ERROR;
  }

  auto& hw_handle = handle(config);
  if (initialized(config)) {
    if (!started(config)) {
      if (HAL_FDCAN_Start(&hw_handle) != HAL_OK) {
        return result::RECOVERABLE_ERROR;
      }
    }

    return refresh_notifications(config);
  }

  __HAL_RCC_FDCAN_CLK_ENABLE();
  init_pin(init_config.port_rx(), init_config.rx_pin_init);
  init_pin(init_config.port_tx(), init_config.tx_pin_init);

  hw_handle = {};
  const auto current_config = controller_config(config);
  hw_handle.Instance = init_config.instance();
  hw_handle.Init = init_config.init;
  hw_handle.Init.FrameFormat =
      fdcan_controller_frame_format(current_config.max_frame_format);
  hw_handle.Init.StdFiltersNbr = current_config.standard_filter_count;
  hw_handle.Init.ExtFiltersNbr = current_config.extended_filter_count;

  if (HAL_FDCAN_Init(&hw_handle) != HAL_OK) {
    return result::RECOVERABLE_ERROR;
  }

  if (HAL_FDCAN_ConfigTimestampCounter(&hw_handle, 1U) != HAL_OK) {
    return result::RECOVERABLE_ERROR;
  }

  if (apply_global_filter(config) != result::OK) {
    return result::RECOVERABLE_ERROR;
  }

  if (HAL_FDCAN_ConfigInterruptLines(&hw_handle, FDCAN_IT_GROUP_RX_FIFO0,
                                     FDCAN_INTERRUPT_LINE0) != HAL_OK) {
    return result::RECOVERABLE_ERROR;
  }

  if (HAL_FDCAN_ConfigInterruptLines(
          &hw_handle,
          FDCAN_IT_GROUP_RX_FIFO1 | FDCAN_IT_GROUP_SMSG | FDCAN_IT_GROUP_MISC |
              FDCAN_IT_GROUP_BIT_LINE_ERROR | FDCAN_IT_GROUP_PROTOCOL_ERROR,
          FDCAN_INTERRUPT_LINE1) != HAL_OK) {
    return result::RECOVERABLE_ERROR;
  }

  update_irq_priorities(config);
  HAL_NVIC_EnableIRQ(fdcan_it0_irq(config));
  HAL_NVIC_EnableIRQ(fdcan_it1_irq(config));

  if (HAL_FDCAN_Start(&hw_handle) != HAL_OK) {
    return result::RECOVERABLE_ERROR;
  }

  return refresh_notifications(config);
}

result stop_controller(const opaque_can& config) noexcept {
  if (!initialized(config) || !started(config)) {
    return result::OK;
  }

  auto& hw_handle = handle(config);
  if (HAL_FDCAN_DeactivateNotification(
          &hw_handle,
          FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE |
              FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_WARNING | FDCAN_IT_ERROR_PASSIVE |
              FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR |
              FDCAN_IT_RAM_ACCESS_FAILURE | FDCAN_IT_ERROR_LOGGING_OVERFLOW |
              FDCAN_IT_RESERVED_ADDRESS_ACCESS | FDCAN_IT_TX_COMPLETE) != HAL_OK) {
    return result::RECOVERABLE_ERROR;
  }

  if (HAL_FDCAN_Stop(&hw_handle) != HAL_OK) {
    return result::RECOVERABLE_ERROR;
  }

  return result::OK;
}

expected::expected<CanMessageTs, result> read_fifo_message(const opaque_can& config,
                                                          const M_fifo fifo) noexcept {
  if (!initialized(config) || !started(config)) {
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }

  auto& hw_handle = handle(config);
  if (HAL_FDCAN_GetRxFifoFillLevel(&hw_handle, hw_fifo(fifo)) == 0U) {
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }

  FDCAN_RxHeaderTypeDef header{};
  uint8_t payload[k_can_fd_max_payload]{};
  if (HAL_FDCAN_GetRxMessage(&hw_handle, hw_fifo(fifo), &header, payload) != HAL_OK) {
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }

  return make_message_ts(header, payload);
}

result write_controller_message(const opaque_can& config,
                                const CanMessage& message) noexcept {
  if (!initialized(config) || !started(config)) {
    return result::RECOVERABLE_ERROR;
  }
  if (message.len > k_can_classic_max_payload) {
    return result::UNRECOVERABLE_ERROR;
  }

  FDCAN_TxHeaderTypeDef header{};
  header.Identifier = message.id;
  header.IdType = FDCAN_STANDARD_ID;
  header.TxFrameType = FDCAN_DATA_FRAME;
  header.DataLength = fdcan_dlc_from_length(static_cast<uint8_t>(message.len));
  header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  header.BitRateSwitch = FDCAN_BRS_OFF;
  header.FDFormat = FDCAN_CLASSIC_CAN;
  header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  header.MessageMarker = 0U;

  uint8_t payload[k_can_classic_max_payload]{};
  std::memcpy(payload, message.bytes, static_cast<std::size_t>(message.len));
  return from_hal_status(HAL_FDCAN_AddMessageToTxFifoQ(&handle(config), &header, payload));
}

result configure_m_filter(const opaque_can& config, const M_filter& filter, const uint8_t id,
                          const bool enabled) noexcept {
  if (!filter_slot_is_valid(m_filter_format(filter), id, controller_config(config))) {
    return result::UNRECOVERABLE_ERROR;
  }

  FDCAN_FilterTypeDef hw_filter{};
  hw_filter.FilterIndex = id;
  if (!translate_m_filter(filter, hw_filter)) {
    return result::UNRECOVERABLE_ERROR;
  }

  if (!enabled) {
    hw_filter.FilterConfig = FDCAN_FILTER_DISABLE;
  }

  return from_hal_status(HAL_FDCAN_ConfigFilter(&handle(config), &hw_filter));
}

result configure_bx_filter(const opaque_can& config, const Bx_filter& filter, const uint8_t id,
                           const bool enabled) noexcept {
  if (!filter_slot_is_valid(bx_filter_format(filter), id, controller_config(config))) {
    return result::UNRECOVERABLE_ERROR;
  }

  FDCAN_FilterTypeDef hw_filter{};
  hw_filter.FilterIndex = id;
  if (!translate_bx_filter(filter, hw_filter)) {
    return result::UNRECOVERABLE_ERROR;
  }

  if (!enabled) {
    hw_filter.FilterConfig = FDCAN_FILTER_DISABLE;
  }

  return from_hal_status(HAL_FDCAN_ConfigFilter(&handle(config), &hw_filter));
}

void service_rx_fifo(const opaque_can& config, const M_fifo fifo) noexcept {
  const auto callback = rx_callbacks(config)[fifo_index(fifo)];
  if (callback == nullptr) {
    return;
  }

  if (HAL_FDCAN_GetRxFifoFillLevel(&handle(config), hw_fifo(fifo)) != 0U) {
    callback();
  }
}

void service_rx_fifo_callback(FDCAN_HandleTypeDef* const hfdcan,
                              const M_fifo fifo) noexcept {
  if (hfdcan == nullptr) {
    return;
  }

  const auto fdcan1_config = make_opaque(FDCAN1);
  if (hfdcan == &handle(fdcan1_config)) {
    service_rx_fifo(fdcan1_config, fifo);
    return;
  }

#if defined(FDCAN2)
  const auto fdcan2_config = make_opaque(FDCAN2);
  if (hfdcan == &handle(fdcan2_config)) {
    service_rx_fifo(fdcan2_config, fifo);
  }
#endif
}

void service_tx_callback(FDCAN_HandleTypeDef* const hfdcan) noexcept {
  if (hfdcan == nullptr) {
    return;
  }

  const auto fdcan1_config = make_opaque(FDCAN1);
  if (hfdcan == &handle(fdcan1_config) &&
      txfull_callback(fdcan1_config) != nullptr) {
    txfull_callback(fdcan1_config)();
    return;
  }

#if defined(FDCAN2)
  const auto fdcan2_config = make_opaque(FDCAN2);
  if (hfdcan == &handle(fdcan2_config) &&
      txfull_callback(fdcan2_config) != nullptr) {
    txfull_callback(fdcan2_config)();
  }
#endif
}

void handle_interrupt(FDCAN_GlobalTypeDef* const p_instance) noexcept {
  auto& hw_handle = handle(make_opaque(p_instance));
  HAL_FDCAN_IRQHandler(&hw_handle);
}
}  // namespace ru::driver::can_internal

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan,
                                          uint32_t rx_fifo0_its) {
  (void)rx_fifo0_its;
  ru::driver::can_internal::service_rx_fifo_callback(hfdcan,
                                                     ru::driver::M_fifo::FIFO0);
}

extern "C" void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* hfdcan,
                                          uint32_t rx_fifo1_its) {
  (void)rx_fifo1_its;
  ru::driver::can_internal::service_rx_fifo_callback(hfdcan,
                                                     ru::driver::M_fifo::FIFO1);
}

extern "C" void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef* hfdcan,
                                                   uint32_t buffer_indexes) {
  (void)buffer_indexes;
  ru::driver::can_internal::service_tx_callback(hfdcan);
}

extern "C" void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef* hfdcan,
                                              uint32_t error_status_its) {
  if (hfdcan == nullptr || hfdcan->Instance == nullptr) {
    return;
  }

  FDCAN_ProtocolStatusTypeDef protocol_status{};
  if (HAL_FDCAN_GetProtocolStatus(hfdcan, &protocol_status) != HAL_OK) {
    return;
  }

  if ((error_status_its & FDCAN_IT_BUS_OFF) != 0U ||
      protocol_status.BusOff != 0U) {
    CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
  }
}

extern "C" void FDCAN1_IT0_IRQHandler(void) {
  ru::driver::can_internal::handle_interrupt(FDCAN1);
}

extern "C" void FDCAN1_IT1_IRQHandler(void) {
  ru::driver::can_internal::handle_interrupt(FDCAN1);
}

#if defined(FDCAN2)
extern "C" void FDCAN2_IT0_IRQHandler(void) {
  ru::driver::can_internal::handle_interrupt(FDCAN2);
}

extern "C" void FDCAN2_IT1_IRQHandler(void) {
  ru::driver::can_internal::handle_interrupt(FDCAN2);
}
#endif
