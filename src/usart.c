#include "usart.h"
#include "stdbool.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <misc.h>
#include "diag/Trace.h"
#include <stdio.h>
#include <math.h>
#include "string.h"

void initUsart(void){

	USART_Cmd(USART1, ENABLE);

	USART_InitTypeDef usartConfig;
	usartConfig.USART_BaudRate = 115200;
	usartConfig.USART_WordLength = USART_WordLength_8b;
	usartConfig.USART_StopBits = USART_StopBits_1;
	usartConfig.USART_Parity = USART_Parity_No;
	usartConfig.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	usartConfig.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART1, &usartConfig);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    /* Enable USART1 global interrupt */
    NVIC_EnableIRQ(USART1_IRQn);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

}



void USART1_PutChar(int c)
{
	// Wait until transmit data register is empty
	while (!USART_GetFlagStatus(USART1, USART_FLAG_TXE));
	// Send a char using USART2
	USART_SendData(USART1, c);
}

void SendData(uint8_t *buf, uint16_t NumBytes){
	for (int i  = 0; i < NumBytes; i++){
		while (!USART_GetFlagStatus(USART1, USART_FLAG_TXE));
		USART_SendData(USART1, *buf);
		*buf++;
	}
	return;
}
