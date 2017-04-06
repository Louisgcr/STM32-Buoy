#ifndef USART_H_
#define USART_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "diag/Trace.h"

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include <stm32f10x_usart.h>


void initUsart(void);
void USART1_PutChar(int c);

void SendData(uint8_t *buf, uint16_t NumBytes);

#endif
