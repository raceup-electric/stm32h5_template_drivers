#pragma once

#include <stddef.h>
#include <stdint.h>

#include "stm32h5xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

int ru_stm32_usb_cdc_start(void);
int ru_stm32_usb_cdc_init(void);
int ru_stm32_usb_cdc_stop(void);
void ru_stm32_usb_cdc_tasks_run(void);
int ru_stm32_usb_cdc_is_configured(void);
int ru_stm32_usb_cdc_raw_try_write(const uint8_t* data, size_t len);
int ru_stm32_usb_cdc_write(const uint8_t* data, size_t len, uint64_t timeout_us);
int ru_stm32_usb_cdc_try_write(const uint8_t* data, size_t len);
uint32_t ru_stm32_usb_cdc_service_task_priority_get(void);
uint32_t ru_stm32_usb_cdc_service_task_period_get(void);

extern PCD_HandleTypeDef hpcd_USB_DRD_FS;

#ifdef __cplusplus
}
#endif
