
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "diag/Trace.h"

#include "stm32f10x.h"
//Import USART to communicate with CPU
#include "stm32f10x_usart.h"

#include "PPM.h"
#include "I2C_Master.h"
#include "MPU9250.h"
#include "delay.h"
#include "usart.h"
#include <misc.h>

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

#define buf_size 256
#define max_msg_size 24
#define min_msg_size 6

#define CPU_CORE_FREQUENCY_HZ 72000000 /* CPU core frequency in Hz */

//Global Variables
volatile uint16_t head, tail;//Head and tail of Ring buffer
uint8_t buffer[buf_size]; //Ring buffer size 256

int throttle1, throttle2, throttle3; //Keep track of throttle level
int newthrottle1, newthrottle2, newthrottle3;

int USART1_IRQHandler(void);
void Ring_Buf_Get(void);
void interpret(void);

//Init functions sets all the Pins to the various mode
int init(void){

	//Initializing the clocks
	//USART, I2C, TIM3, GPIOA, GPIOB and AFIO clocks enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA |RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_I2C1, ENABLE);

	//Configuration of PPM GPIO pins
	PWM_Pin_Configuration();
	//Configuration PPM for 72Mhz Clock
	Tim_Config();
	initUsart();
	I2C_Master_Init();
	DelayInit();

	return 0;
}

// ----- main() ---------------------------------------------------------------
int main(int argc, char* argv[]){

	//Low level initialization of pins
	init();

	//Enable Sensors
	calibrate_sensor();
	init_sensor();

	//Keep track of throttle level
	throttle1 = throttle2 = throttle3 = 0;
	newthrottle1 = newthrottle2 = newthrottle3 = 4500;
	mov(throttle1,newthrottle1,throttle2,newthrottle2,throttle3,newthrottle3);
	throttle1 = throttle2 = throttle3 = 4500;

	//Head and tail of ring buffer
	head=0;
	tail=0;
	memset(buffer,0,sizeof(buffer));
	magMaxX = magMaxY = magMaxZ = magMinX = magMinY = magMinZ = 0.0;
	while (1){

		Ring_Buf_Get();
		read_acc();
		read_mag();
		read_gyro();
		AHRS_Send();
		//run_mag_calibration();

	}
}

int USART1_IRQHandler(void){
	//If buffer is not full
	if( !(head - tail == buf_size)){
		//Important to keep uint8_t as it truncates head to give value between 0 - 255
		buffer[(uint8_t)head] = (uint8_t)USART_ReceiveData(USART1);
		head++;
		return 0;
	}else{
		//TODO: Throw error when return -1
		return -1;
	}

}

void Ring_Buf_Get(void){

	//offset here is only uint8_t as buffer size is cap at 256
	uint8_t offset;

	//This gives you the absolute offset between head and tail
	offset = head - tail;

	//If buffer is not empty
	if((uint8_t)offset >= min_msg_size){
		//Check if start condition is met

		if(buffer[(uint8_t)tail] == 255 && buffer[(uint8_t)(tail+1)] == 255){//(uint8_t)offset > max_msg_size &&
			//interpret instruction
			//uint8_t test[18];
			interpret();
			tail = tail+2+buffer[(uint8_t)(tail+2)];

		}else{
			//increase pointer until start condition is met and wait for more data
			tail++;
		}
	}

	return ;

}


void interpret(void){

	//Use switch if there is more if condition
	if(buffer[(uint8_t)(tail+3)] == 1){

		newthrottle1 = buffer[(uint8_t)(tail+4)]<<8 | buffer[(uint8_t)(tail+5)];
		newthrottle2 = buffer[(uint8_t)(tail+6)]<<8 | buffer[(uint8_t)(tail+7)];
		newthrottle3 = buffer[(uint8_t)(tail+8)]<<8 | buffer[(uint8_t)(tail+9)];

		mov(throttle1,newthrottle1,throttle2,newthrottle2,throttle3,newthrottle3);
		throttle1 = newthrottle1;
		throttle2 = newthrottle2;
		throttle3 = newthrottle3;

	}
	return;
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------

