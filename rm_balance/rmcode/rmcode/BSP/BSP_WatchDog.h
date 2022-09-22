#ifndef __BSP_WATCHDOG_H
#define __BSP_WATCHDOG_H

#include "stm32f4xx.h"
#include "stdlib.h"
#include "stdio.h"

void BSP_IWDG_Init(uint8_t prer,uint16_t rlr);
void BSP_IWDG_Feed(void);
void BSP_WWDG_Init(uint8_t tr,uint8_t wr,uint32_t fprer,uint8_t PreemptionPriority,uint8_t SubPriority);

#endif
