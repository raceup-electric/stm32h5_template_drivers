#ifndef STM32H5xx_HAL_CONF_H
#define STM32H5xx_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#define HAL_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_EXTI_MODULE_ENABLED
#define HAL_FDCAN_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_IWDG_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_PCD_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED

#define USE_HAL_ADC_REGISTER_CALLBACKS 0U
#define USE_HAL_FDCAN_REGISTER_CALLBACKS 0U
#define USE_HAL_IWDG_REGISTER_CALLBACKS 0U
#define USE_HAL_PCD_REGISTER_CALLBACKS 0U
#define USE_HAL_TIM_REGISTER_CALLBACKS 0U
#define USE_HAL_UART_REGISTER_CALLBACKS 0U

#if !defined(HSE_VALUE)
#define HSE_VALUE 12000000U
#endif

#if !defined(HSE_STARTUP_TIMEOUT)
#define HSE_STARTUP_TIMEOUT 100UL
#endif

#if !defined(CSI_VALUE)
#define CSI_VALUE 4000000UL
#endif

#if !defined(HSI_VALUE)
#define HSI_VALUE 64000000UL
#endif

#if !defined(HSI48_VALUE)
#define HSI48_VALUE 48000000UL
#endif

#if !defined(LSI_VALUE)
#define LSI_VALUE 32000UL
#endif

#if !defined(LSI_STARTUP_TIME)
#define LSI_STARTUP_TIME 130UL
#endif

#if !defined(LSE_VALUE)
#define LSE_VALUE 32768UL
#endif

#if !defined(LSE_STARTUP_TIMEOUT)
#define LSE_STARTUP_TIMEOUT 5000UL
#endif

#if !defined(EXTERNAL_CLOCK_VALUE)
#define EXTERNAL_CLOCK_VALUE 12288000UL
#endif

#define VDD_VALUE 3300UL
#define TICK_INT_PRIORITY ((1UL << __NVIC_PRIO_BITS) - 1UL)
#define USE_RTOS 0U
#define PREFETCH_ENABLE 0U

#ifdef HAL_RCC_MODULE_ENABLED
#include "stm32h5xx_hal_rcc.h"
#endif

#ifdef HAL_DMA_MODULE_ENABLED
#include "stm32h5xx_hal_dma.h"
#endif

#ifdef HAL_ADC_MODULE_ENABLED
#include "stm32h5xx_hal_adc.h"
#include "stm32h5xx_hal_adc_ex.h"
#endif

#ifdef HAL_GPIO_MODULE_ENABLED
#include "stm32h5xx_hal_gpio.h"
#include "stm32h5xx_hal_gpio_ex.h"
#endif

#ifdef HAL_CORTEX_MODULE_ENABLED
#include "stm32h5xx_hal_cortex.h"
#endif

#ifdef HAL_FDCAN_MODULE_ENABLED
#include "stm32h5xx_hal_fdcan.h"
#endif

#ifdef HAL_FLASH_MODULE_ENABLED
#include "stm32h5xx_hal_flash.h"
#endif

#ifdef HAL_IWDG_MODULE_ENABLED
#include "stm32h5xx_hal_iwdg.h"
#endif

#ifdef HAL_PWR_MODULE_ENABLED
#include "stm32h5xx_hal_pwr.h"
#endif

#ifdef HAL_EXTI_MODULE_ENABLED
#include "stm32h5xx_hal_exti.h"
#endif

#ifdef HAL_TIM_MODULE_ENABLED
#include "stm32h5xx_hal_tim.h"
#endif

#ifdef HAL_UART_MODULE_ENABLED
#include "stm32h5xx_hal_uart.h"
#include "stm32h5xx_hal_uart_ex.h"
#endif

#ifdef HAL_PCD_MODULE_ENABLED
#if defined(__GNUC__) && defined(__cplusplus)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wvolatile"
#endif
#include "stm32h5xx_hal_pcd.h"
#include "stm32h5xx_hal_pcd_ex.h"
#if defined(__GNUC__) && defined(__cplusplus)
#pragma GCC diagnostic pop
#endif
#endif

#ifdef USE_FULL_ASSERT
#define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
void assert_failed(uint8_t *file, uint32_t line);
#else
#define assert_param(expr) ((void)0U)
#endif

#ifdef __cplusplus
}
#endif

#endif
