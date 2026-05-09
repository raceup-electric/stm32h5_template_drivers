#include "ux_api.h"

#include "stm32h5xx_hal.h"

ALIGN_TYPE _ux_utility_interrupt_disable(VOID)
{
  const ALIGN_TYPE primask = (ALIGN_TYPE)__get_PRIMASK();
  __disable_irq();
  return primask;
}

VOID _ux_utility_interrupt_restore(ALIGN_TYPE flags)
{
  if ((flags & 1U) == 0U)
  {
    __enable_irq();
  }
  else
  {
    __disable_irq();
  }
}

ULONG _ux_utility_time_get(VOID)
{
  return (ULONG)HAL_GetTick();
}
