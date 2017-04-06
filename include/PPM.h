#ifndef PPM_H_
#define PPM_H_

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"

void Tim_Config(void);
void PWM_Pin_Configuration(void);
void Drive_motor(int value, uint8_t motor);
int increment(int throttleOld, int throttleNew, uint8_t motor);
void acc(int throttleOld, int throttle, uint8_t motor);
void mov(int current1, int input1, int current2, int input2, int current3, int input3);

// ----------------------------------------------------------------------------

#endif
