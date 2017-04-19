#include "PPM.h"
#include "math.h"

//TODO Check if the rate can be lowered
#define RATE 0.01

void Tim_Config(void){

	   /*
	   * Calculation of Prescaler
	   * STM32 TIM3 Clock 72Mhz
	   * PPM Frequency = 50Hz
	   * Period = ((72000000/Prescaler)/Frequency)
	   * Prescaler = 24
	   * Prescaler & Period Values needs to be both below 65536
	   * Prescaler was chosen so that Period has a higher resolution
	   */

	  /**** Timer3 time base setup ****/
	  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	  TIM_TimeBaseInitStructure.TIM_Prescaler=24-1;
	  TIM_TimeBaseInitStructure.TIM_Period =60000;
	  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up ;
	  TIM_TimeBaseInitStructure.TIM_ClockDivision = 0;
	  TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
	  TIM_Cmd(TIM3, ENABLE);

	  TIM_OCInitTypeDef TIM_OCInitStructure;
	  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	  TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable ;
	  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	  TIM_OC1Init(TIM3, &TIM_OCInitStructure); //PA6 - ouptut pwm on channel 1
	  TIM_OC2Init(TIM3, &TIM_OCInitStructure); //PA7 - ouptut pwm on channel 2
	  TIM_OC3Init(TIM3, &TIM_OCInitStructure); //PB0 - ouptut pwm on channel 3
	  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);//Load configuration to Timer 3

}


void PWM_Pin_Configuration(void){

	  /*
	   * GPIO Pins are configured based on pins*/
	  GPIO_InitTypeDef GPIO_InitStructure;

	  /*-------------------------- GPIO Configuration ----------------------------*/
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);
}


/*
 * Function to drive PPM Channels CCRx, x = which channel(1,2,3,...) to drive.
 * Arguments:  value = Current throttle
 * 			   motor  	   = Motor channel
 *  */
void Drive_motor(int value, uint8_t motor){
	/*
	 *Period = 20ms, 3000 points of resolution
	 *1ms = 4500
	 *Min value = 1ms = 3000
	 *Max value = 2ms = 6000
	 */
	if( motor & 0b00000001){
		TIM3->CCR1=value; // just some duty cycle value
	}

	if( motor & 0b00000010){
		TIM3->CCR2=value; // just some duty cycle value
	}

	if( motor & 0b00000100){
		TIM3->CCR3=value; // just some duty cycle value
	}

}

/*
 * Increment Step size of each throttle increment.
 * Arguments:  throttleOld = Current throttle
 * 			   throttleNew = Desired throttle
 * 			   motor  	   = Motor channel
 *  */
int increment(int throttleOld, int throttleNew, uint8_t motor){
	int i = throttleOld;
	if (throttleNew > throttleOld){
		i = i + ceil(RATE * (throttleNew - throttleOld));
		Drive_motor(i, motor);
	}else if (throttleOld > throttleNew){
		i = i - ceil(RATE * (throttleOld - throttleNew));
		Drive_motor(i, motor);
	}


	return i;
}

/*
 * Increment throttle increases the throttle rate recursively until throttle value reaches the desired throttle value.
 * Arguments:  throttleOld = Current throttle
 * 			   throttleNew = Desired throttle
 * 			   motor  	   = Motor channel
 *  */
void acc(int throttleOld, int throttleNew, uint8_t motor){

	if (throttleOld != throttleNew ){
		acc(increment(throttleOld, throttleNew, motor), throttleNew, motor);
		return;
	}

	return;
}

/*
 * Helper function to move motors to desired speed
 * Arguments:  current1 = Current throttle of motor 1
 * 			   input1   = Desired throttle of motor 1
 *			   current2 = Current throttle of motor 2
 * 			   input2   = Desired throttle of motor 2
 * 			   current3 = Current throttle of motor 3
 * 			   input3   = Desired throttle of motor 3
 *  */
void mov(int current1, int input1, int current2, int input2, int current3, int input3){

	acc(current1, input1, 0b00000001);
	acc(current2, input2, 0b00000010);
	acc(current3, input3, 0b00000100);

}
