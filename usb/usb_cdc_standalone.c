#include "usb_cdc.h"

#include <stdio.h>
#include <string.h>

#include "ux_api.h"
#include "ux_device_class_cdc_acm.h"
#include "ux_device_stack.h"
#include "ux_dcd_stm32.h"
#include "ux_system.h"

#define RU_USBX_MEMORY_SIZE (12U * 1024U)
#define RU_USB_VID 0x0483U
#define RU_USB_PID 0x5710U
#define RU_USB_BCD_DEVICE 0x0100U
#define RU_USB_CDC_STANDALONE_TX_BUFFER_SIZE 128U

#ifndef RU_USB_MANUFACTURER_STRING
#define RU_USB_MANUFACTURER_STRING "Template"
#endif

#ifndef RU_USB_PRODUCT_STRING
#define RU_USB_PRODUCT_STRING "USB CDC"
#endif

typedef struct {
  UX_SLAVE_CLASS_CDC_ACM* instance;
  UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_PARAMETER line_coding;
  UX_SLAVE_CLASS_CDC_ACM_LINE_STATE_PARAMETER line_state;
  ULONG tx_length;
  ULONG tx_actual_length;
  uint8_t tx_active;
  uint8_t started;
  uint8_t tx_buffer[RU_USB_CDC_STANDALONE_TX_BUFFER_SIZE];
} ru_usb_cdc_standalone_context_t;

PCD_HandleTypeDef hpcd_USB_DRD_FS __attribute__((weak));

static ru_usb_cdc_standalone_context_t g_usb_cdc_standalone = {
    .line_coding =
        {
            .ux_slave_class_cdc_acm_parameter_baudrate = 115200UL,
            .ux_slave_class_cdc_acm_parameter_stop_bit = 0U,
            .ux_slave_class_cdc_acm_parameter_parity = 0U,
            .ux_slave_class_cdc_acm_parameter_data_bit = 8U,
        },
};

__ALIGN_BEGIN static uint8_t g_usbx_memory[RU_USBX_MEMORY_SIZE] __ALIGN_END;

static UCHAR g_language_id_framework[] = {0x09U, 0x04U};
static UCHAR g_string_framework[96];
static ULONG g_string_framework_length;

static UCHAR g_device_framework_full_speed[] = {
    0x12, 0x01, 0x00, 0x02, 0xEF, 0x02, 0x01, 0x40,
    (UCHAR)(RU_USB_VID & 0xFFU), (UCHAR)(RU_USB_VID >> 8),
    (UCHAR)(RU_USB_PID & 0xFFU), (UCHAR)(RU_USB_PID >> 8),
    (UCHAR)(RU_USB_BCD_DEVICE & 0xFFU), (UCHAR)(RU_USB_BCD_DEVICE >> 8),
    0x01, 0x02, 0x03, 0x01,

    0x09, 0x02, 0x4BU, 0x00, 0x02, 0x01, 0x00, 0x80, 0x32,

    0x08, 0x0BU, 0x00, 0x02, 0x02, 0x02, 0x01, 0x00,

    0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00,
    0x05, 0x24, 0x00, 0x10, 0x01,
    0x04, 0x24, 0x02, 0x02,
    0x05, 0x24, 0x06, 0x00, 0x01,
    0x05, 0x24, 0x01, 0x00, 0x01,
    0x07, 0x05, 0x83, 0x03, 0x08, 0x00, 0x10,

    0x09, 0x04, 0x01, 0x00, 0x02, 0x0A, 0x00, 0x00, 0x00,
    0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x00,
    0x07, 0x05, 0x81, 0x02, 0x40, 0x00, 0x00,
};

static UCHAR g_device_framework_high_speed[] = {
    0x12, 0x01, 0x00, 0x02, 0xEF, 0x02, 0x01, 0x40,
    (UCHAR)(RU_USB_VID & 0xFFU), (UCHAR)(RU_USB_VID >> 8),
    (UCHAR)(RU_USB_PID & 0xFFU), (UCHAR)(RU_USB_PID >> 8),
    (UCHAR)(RU_USB_BCD_DEVICE & 0xFFU), (UCHAR)(RU_USB_BCD_DEVICE >> 8),
    0x01, 0x02, 0x03, 0x01,

    0x0A, 0x06, 0x00, 0x02, 0xEF, 0x02, 0x01, 0x40, 0x01, 0x00,

    0x09, 0x02, 0x4BU, 0x00, 0x02, 0x01, 0x00, 0x80, 0x32,

    0x08, 0x0BU, 0x00, 0x02, 0x02, 0x02, 0x01, 0x00,

    0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00,
    0x05, 0x24, 0x00, 0x10, 0x01,
    0x04, 0x24, 0x02, 0x02,
    0x05, 0x24, 0x06, 0x00, 0x01,
    0x05, 0x24, 0x01, 0x00, 0x01,
    0x07, 0x05, 0x83, 0x03, 0x08, 0x00, 0x10,

    0x09, 0x04, 0x01, 0x00, 0x02, 0x0A, 0x00, 0x00, 0x00,
    0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x00,
    0x07, 0x05, 0x81, 0x02, 0x40, 0x00, 0x00,
};

void ru_stm32_usb_cdc_standalone_dcd_tasks_run(void);

static void ru_usb_append_string(UCHAR index, const char* text, size_t* offset) {
  const size_t length = strlen(text);

  if ((*offset + 4U + length) > sizeof(g_string_framework)) {
    return;
  }

  g_string_framework[(*offset)++] = 0x09U;
  g_string_framework[(*offset)++] = 0x04U;
  g_string_framework[(*offset)++] = index;
  g_string_framework[(*offset)++] = (UCHAR)length;
  memcpy(&g_string_framework[*offset], text, length);
  *offset += length;
}

static void ru_usb_build_string_framework(void) {
  char serial_number[25];
  size_t offset = 0U;
  const int serial_length =
      snprintf(serial_number, sizeof(serial_number), "%08lX%08lX%08lX",
               (unsigned long)HAL_GetUIDw0(), (unsigned long)HAL_GetUIDw1(),
               (unsigned long)HAL_GetUIDw2());

  memset(g_string_framework, 0, sizeof(g_string_framework));
  ru_usb_append_string(0x01U, RU_USB_MANUFACTURER_STRING, &offset);
  ru_usb_append_string(0x02U, RU_USB_PRODUCT_STRING, &offset);
  ru_usb_append_string(0x03U, (serial_length > 0) ? serial_number : "00000000",
                       &offset);
  g_string_framework_length = (ULONG)offset;
}

static VOID ru_usb_cdc_activate(VOID* cdc_instance) {
  g_usb_cdc_standalone.instance = (UX_SLAVE_CLASS_CDC_ACM*)cdc_instance;
#ifndef UX_DEVICE_CLASS_CDC_ACM_TRANSMISSION_DISABLE
  g_usb_cdc_standalone.instance->ux_slave_class_cdc_acm_transmission_status =
      UX_FALSE;
  g_usb_cdc_standalone.instance->ux_slave_class_cdc_acm_scheduled_write =
      UX_FALSE;
  g_usb_cdc_standalone.instance->ux_device_class_cdc_acm_read_state =
      UX_STATE_RESET;
  g_usb_cdc_standalone.instance->ux_device_class_cdc_acm_write_state =
      UX_STATE_RESET;
#endif
  (void)ux_device_class_cdc_acm_ioctl(
      g_usb_cdc_standalone.instance,
      UX_SLAVE_CLASS_CDC_ACM_IOCTL_SET_LINE_CODING,
      &g_usb_cdc_standalone.line_coding);
}

static VOID ru_usb_cdc_deactivate(VOID* cdc_instance) {
  UX_PARAMETER_NOT_USED(cdc_instance);
  g_usb_cdc_standalone.instance = UX_NULL;
}

static VOID ru_usb_cdc_parameter_change(VOID* cdc_instance) {
  UX_PARAMETER_NOT_USED(cdc_instance);

  if (g_usb_cdc_standalone.instance == UX_NULL) {
    return;
  }

  (void)ux_device_class_cdc_acm_ioctl(
      g_usb_cdc_standalone.instance,
      UX_SLAVE_CLASS_CDC_ACM_IOCTL_GET_LINE_CODING,
      &g_usb_cdc_standalone.line_coding);
  (void)ux_device_class_cdc_acm_ioctl(
      g_usb_cdc_standalone.instance,
      UX_SLAVE_CLASS_CDC_ACM_IOCTL_GET_LINE_STATE,
      &g_usb_cdc_standalone.line_state);
}

int ru_stm32_usb_cdc_standalone_stack_only_start(void) {
  UX_SLAVE_CLASS_CDC_ACM_PARAMETER parameter;

  if (g_usb_cdc_standalone.started != 0U) {
    return 0;
  }

  memset(&parameter, 0, sizeof(parameter));
  memset(&hpcd_USB_DRD_FS, 0, sizeof(hpcd_USB_DRD_FS));
  ru_usb_build_string_framework();

  if (ux_system_initialize(g_usbx_memory, sizeof(g_usbx_memory), UX_NULL, 0U) !=
      UX_SUCCESS) {
    return -1;
  }

  if (ux_device_stack_initialize(
          g_device_framework_high_speed, sizeof(g_device_framework_high_speed),
          g_device_framework_full_speed, sizeof(g_device_framework_full_speed),
          g_string_framework, g_string_framework_length,
          g_language_id_framework, sizeof(g_language_id_framework),
          UX_NULL) != UX_SUCCESS) {
    return -1;
  }

  parameter.ux_slave_class_cdc_acm_instance_activate = ru_usb_cdc_activate;
  parameter.ux_slave_class_cdc_acm_instance_deactivate = ru_usb_cdc_deactivate;
  parameter.ux_slave_class_cdc_acm_parameter_change =
      ru_usb_cdc_parameter_change;

  if (ux_device_stack_class_register(_ux_system_slave_class_cdc_acm_name,
                                     ux_device_class_cdc_acm_entry, 1U, 0U,
                                     &parameter) != UX_SUCCESS) {
    return -1;
  }

  g_usb_cdc_standalone.started = 1U;
  return 0;
}

static int ru_usb_cdc_standalone_configure_pcd(void) {
  hpcd_USB_DRD_FS.Instance = USB_DRD_FS;
  hpcd_USB_DRD_FS.Init.dev_endpoints = 8U;
  hpcd_USB_DRD_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_DRD_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_DRD_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.bulk_doublebuffer_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.iso_singlebuffer_enable = DISABLE;

  if (HAL_PCD_Init(&hpcd_USB_DRD_FS) != HAL_OK) {
    return -1;
  }

  if (HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, 0x00U, PCD_SNG_BUF, 0x20U) !=
          HAL_OK ||
      HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, 0x80U, PCD_SNG_BUF, 0x60U) !=
          HAL_OK ||
      HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, 0x81U, PCD_SNG_BUF, 0xA0U) !=
          HAL_OK ||
      HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, 0x02U, PCD_SNG_BUF, 0xE0U) !=
          HAL_OK ||
      HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, 0x83U, PCD_SNG_BUF, 0x120U) !=
          HAL_OK) {
    return -1;
  }

  return 0;
}

int ru_stm32_usb_cdc_standalone_dcd_only_start(void) {
  if (ru_stm32_usb_cdc_standalone_stack_only_start() != 0) {
    return -1;
  }

  if (ru_usb_cdc_standalone_configure_pcd() != 0) {
    return -1;
  }

  if (_ux_dcd_stm32_initialize((ULONG)USB_DRD_FS,
                               (ULONG)&hpcd_USB_DRD_FS) != UX_SUCCESS) {
    return -1;
  }

  return 0;
}

int ru_stm32_usb_cdc_standalone_start(void) {
  if (ru_stm32_usb_cdc_standalone_dcd_only_start() != 0) {
    return -1;
  }

  if (HAL_PCD_Start(&hpcd_USB_DRD_FS) != HAL_OK) {
    return -1;
  }

  return 0;
}

static int ru_usb_cdc_standalone_is_configured(void) {
  return (g_usb_cdc_standalone.instance != UX_NULL &&
          g_usb_cdc_standalone.instance->ux_slave_class_cdc_acm_interface !=
              UX_NULL &&
          _ux_system_slave->ux_system_slave_device.ux_slave_device_state ==
              UX_DEVICE_CONFIGURED)
             ? 1
             : 0;
}

static void ru_usb_cdc_standalone_service_tx(void) {
  UINT status;

  if (g_usb_cdc_standalone.tx_active == 0U ||
      ru_usb_cdc_standalone_is_configured() == 0) {
    return;
  }

  status = ux_device_class_cdc_acm_write_run(
      g_usb_cdc_standalone.instance, g_usb_cdc_standalone.tx_buffer,
      g_usb_cdc_standalone.tx_length,
      &g_usb_cdc_standalone.tx_actual_length);

  if (status == UX_STATE_NEXT) {
    g_usb_cdc_standalone.tx_active = 0U;
    g_usb_cdc_standalone.tx_length = 0U;
  } else if (status < UX_STATE_NEXT) {
    g_usb_cdc_standalone.tx_active = 0U;
    g_usb_cdc_standalone.tx_length = 0U;
    if (g_usb_cdc_standalone.instance != UX_NULL) {
      g_usb_cdc_standalone.instance->ux_device_class_cdc_acm_write_state =
          UX_STATE_RESET;
    }
  }
}

void ru_stm32_usb_cdc_standalone_tasks_run(void) {
  UX_SLAVE_CLASS* class_instance;
  ULONG class_index;

  if (g_usb_cdc_standalone.started == 0U) {
    return;
  }

  ru_stm32_usb_cdc_standalone_dcd_tasks_run();

  if (g_usb_cdc_standalone.instance == UX_NULL ||
      g_usb_cdc_standalone.instance->ux_slave_class_cdc_acm_interface ==
          UX_NULL) {
    return;
  }

  class_instance = _ux_system_slave->ux_system_slave_class_array;
  for (class_index = 0U; class_index < UX_SYSTEM_DEVICE_MAX_CLASS_GET();
       ++class_index) {
    if (class_instance->ux_slave_class_status != UX_UNUSED &&
        class_instance->ux_slave_class_task_function != UX_NULL) {
      (void)class_instance->ux_slave_class_task_function(
          class_instance->ux_slave_class_instance);
    }

    ++class_instance;
  }

  ru_usb_cdc_standalone_service_tx();
}

void ru_stm32_usb_cdc_standalone_dcd_tasks_run(void) {
  UX_SLAVE_DCD* dcd;

  if (g_usb_cdc_standalone.started == 0U) {
    return;
  }

  dcd = &_ux_system_slave->ux_system_slave_dcd;
  (void)dcd->ux_slave_dcd_function(dcd, UX_DCD_TASKS_RUN, UX_NULL);
}

int ru_stm32_usb_cdc_standalone_is_active(void) {
  return (g_usb_cdc_standalone.instance != UX_NULL) ? 1 : 0;
}

int ru_stm32_usb_cdc_standalone_is_configured(void) {
  return ru_usb_cdc_standalone_is_configured();
}

int ru_stm32_usb_cdc_standalone_write(const uint8_t* data, size_t length) {
  if (data == NULL || length == 0U ||
      ru_usb_cdc_standalone_is_configured() == 0 ||
      g_usb_cdc_standalone.tx_active != 0U) {
    return -1;
  }

  if (length > sizeof(g_usb_cdc_standalone.tx_buffer)) {
    length = sizeof(g_usb_cdc_standalone.tx_buffer);
  }

  memcpy(g_usb_cdc_standalone.tx_buffer, data, length);
  g_usb_cdc_standalone.tx_length = (ULONG)length;
  g_usb_cdc_standalone.tx_actual_length = 0U;
  g_usb_cdc_standalone.tx_active = 1U;
  return 0;
}

int ru_stm32_usb_cdc_start(void) {
  return ru_stm32_usb_cdc_standalone_start();
}

int ru_stm32_usb_cdc_init(void) {
  return g_usb_cdc_standalone.started != 0U ? 0 : -1;
}

int ru_stm32_usb_cdc_stop(void) {
  return 0;
}

void ru_stm32_usb_cdc_tasks_run(void) {
  ru_stm32_usb_cdc_standalone_tasks_run();
}

int ru_stm32_usb_cdc_is_configured(void) {
  return ru_stm32_usb_cdc_standalone_is_configured();
}

int ru_stm32_usb_cdc_raw_try_write(const uint8_t* data, size_t len) {
  return ru_stm32_usb_cdc_standalone_write(data, len);
}

static uint32_t ru_usb_timeout_us_to_ms(uint64_t timeout_us) {
  if (timeout_us == 0U) {
    return 0U;
  }

  if (timeout_us > ((uint64_t)UINT32_MAX * 1000ULL)) {
    return UINT32_MAX;
  }

  const uint64_t timeout_ms = (timeout_us + 999ULL) / 1000ULL;
  return (uint32_t)timeout_ms;
}

int ru_stm32_usb_cdc_write(const uint8_t* data, size_t len,
                           uint64_t timeout_us) {
  if (data == NULL && len != 0U) {
    return -1;
  }

  if (len == 0U) {
    ru_stm32_usb_cdc_tasks_run();
    return 0;
  }

  const uint32_t timeout_ms = ru_usb_timeout_us_to_ms(timeout_us);
  const uint32_t start_ms = HAL_GetTick();

  do {
    ru_stm32_usb_cdc_tasks_run();

    if (ru_stm32_usb_cdc_raw_try_write(data, len) == 0) {
      ru_stm32_usb_cdc_tasks_run();
      return 0;
    }
  } while (timeout_ms != 0U &&
           (uint32_t)(HAL_GetTick() - start_ms) < timeout_ms);

  return -1;
}

int ru_stm32_usb_cdc_try_write(const uint8_t* data, size_t len) {
  return ru_stm32_usb_cdc_write(data, len, 0U);
}
