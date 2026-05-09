#include "main.h"

void HAL_MspInit(void)
{

}

#ifdef HAL_PCD_MODULE_ENABLED
void HAL_PCD_MspInit(PCD_HandleTypeDef* hpcd)
{
  RCC_PeriphCLKInitTypeDef periph_clk_init = {0};

  if (hpcd->Instance != USB_DRD_FS)
  {
    return;
  }

  periph_clk_init.PeriphClockSelection = RCC_PERIPHCLK_USB;
  periph_clk_init.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&periph_clk_init) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_PWREx_EnableVddUSB();

  __HAL_RCC_USB_CLK_ENABLE();

  HAL_NVIC_SetPriority(USB_DRD_FS_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(USB_DRD_FS_IRQn);
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef* hpcd)
{
  if (hpcd->Instance != USB_DRD_FS)
  {
    return;
  }

  __HAL_RCC_USB_CLK_DISABLE();
  HAL_NVIC_DisableIRQ(USB_DRD_FS_IRQn);
}
#endif
