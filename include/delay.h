#ifndef DELAY_H_
#define DELAY_H_

#include "stm32f10x.h"

void DelayInit(void);
void DelayUs(uint32_t us);
void DelayMs(uint32_t ms);

#endif
