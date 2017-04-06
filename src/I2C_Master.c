#include "I2C_Master.h"

#define I2C_MEMS                I2C1
#define I2C_MEMS_CLK            RCC_APB1Periph_I2C1
#define I2C_MEMS_GPIO           GPIOB
#define I2C_MEMS_GPIO_CLK       RCC_APB2Periph_GPIOB
#define I2C_MEMS_SCL            GPIO_Pin_6
#define I2C_MEMS_SDA            GPIO_Pin_7

//MPU Supports fast i2c protocol
#define I2C_Speed               100000//100000
#define I2C_SLAVE_ADDRESS7      0x00
#define I2C_TIMEOUT             3000

/* I2C STOP mask */
#define CR1_STOP_Set            ((uint16_t)0x0200)
#define CR1_STOP_Reset          ((uint16_t)0xFDFF)

/* I2C ACK mask */
#define CR1_ACK_Set             ((uint16_t)0x0400)
#define CR1_ACK_Reset           ((uint16_t)0xFBFF)

/* I2C POS mask */
#define CR1_POS_Set             ((uint16_t)0x0800)
#define CR1_POS_Reset           ((uint16_t)0xF7FF)

#define NULL ((void *)0)


void I2C_Master_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  I2C_InitTypeDef  I2C_InitStructure;

  /* I2C Periph clock enable */
  RCC_APB1PeriphClockCmd(I2C_MEMS_CLK, ENABLE);

  /* GPIO Periph clock enable */
  RCC_APB2PeriphClockCmd(I2C_MEMS_GPIO_CLK, ENABLE);

  /* GPIO configuration */
  GPIO_InitStructure.GPIO_Pin =  I2C_MEMS_SCL | I2C_MEMS_SDA;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(I2C_MEMS_GPIO, &GPIO_InitStructure);

   /* I2C configuration */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = I2C_SLAVE_ADDRESS7;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = I2C_Speed;

  /* I2C Peripheral Enable */
  I2C_Cmd(I2C_MEMS, ENABLE);
  /* Apply I2C configuration after enabling it */
  I2C_Init(I2C_MEMS, &I2C_InitStructure);
}

void I2C_Master_DeInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* UnConfigure I2C */
  I2C_DeInit(I2C_MEMS);
  I2C_Cmd(I2C_MEMS, DISABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, DISABLE);

  /* UnConfigure I2C_MEMS pins: SCL and SDA */
  GPIO_InitStructure.GPIO_Pin = I2C_MEMS_SCL | I2C_MEMS_SDA;
  //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;// Sets GPIO to internal Pull Up
  GPIO_Init(I2C_MEMS_GPIO, &GPIO_InitStructure);

}


int I2C_Master_Read(uint8_t deviceAddr, uint8_t readAddr, uint8_t* pBuffer, uint16_t numByteToRead) {

  __IO uint32_t temp = 0;
  volatile int I2C_TimeOut = 0;

   /* While the bus is busy */
  I2C_TimeOut = I2C_TIMEOUT;
  while(I2C_GetFlagStatus(I2C_MEMS, I2C_FLAG_BUSY))
  {
	  if (I2C_TimeOut-- <= 0){
		  return(I2C_TIMEOUT_UserCallback());
	  }
  }

  /* Send START condition */
  I2C_GenerateSTART(I2C_MEMS, ENABLE);

  /* Test on EV5 and clear it */
  I2C_TimeOut = I2C_TIMEOUT;
  while(!I2C_CheckEvent(I2C_MEMS, I2C_EVENT_MASTER_MODE_SELECT)){
	  if (I2C_TimeOut-- <= 0){
		  return(I2C_TIMEOUT_UserCallback());
	  }
  }

  // / * Send EEPROM address for write  * /
  I2C_Send7bitAddress(I2C_MEMS, deviceAddr, I2C_Direction_Transmitter);

  // / * Test on EV6 and clear it * /
  I2C_TimeOut = I2C_TIMEOUT;
  while(!I2C_CheckEvent(I2C_MEMS, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
	  if (I2C_TimeOut-- <= 0){
		  return(I2C_TIMEOUT_UserCallback());
	  }
  }

  // / * Send the EEPROM's internal address to read from: Only one byte address  * /
  I2C_SendData(I2C_MEMS, readAddr);


  /// * Test on EV8 and clear it * /
  I2C_TimeOut = I2C_TIMEOUT;
  while(!I2C_CheckEvent(I2C_MEMS, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
	  if (I2C_TimeOut-- <= 0){
		  return(I2C_TIMEOUT_UserCallback());
	  }
  }

  // * Send STRAT condition a second time * /
  I2C_GenerateSTART(I2C_MEMS, ENABLE);

  // * Test on EV5 and clear it * /
  I2C_TimeOut = I2C_TIMEOUT;
  while(!I2C_CheckEvent(I2C_MEMS, I2C_EVENT_MASTER_MODE_SELECT)){
	  if (I2C_TimeOut-- <= 0){
		  return(I2C_TIMEOUT_UserCallback());
	  }
  }

    // * Send EEPROM address for read * /
  I2C_Send7bitAddress(I2C_MEMS, deviceAddr, I2C_Direction_Receiver);

  if (numByteToRead == 1)  {
	  /* Wait until ADDR is set */
	  I2C_TimeOut = I2C_TIMEOUT;
	  while ((I2C_MEMS->SR1&0x0002) != 0x0002)
	  {
		  if (I2C_TimeOut-- <= 0){
			  return(I2C_TIMEOUT_UserCallback());
		  }
	  }

	  /* Clear ACK bit */
	  I2C_MEMS->CR1 &= CR1_ACK_Reset;
	  /* Disable all active IRQs around ADDR clearing and STOP programming because the EV6_3
    	software sequence must complete before the current byte end of transfer */
	  __disable_irq();
	  /* Clear ADDR flag */
	  temp = I2C_MEMS->SR2;
	  /* Program the STOP */
	  I2C_GenerateSTOP(I2C_MEMS, ENABLE);
	  /* Re-enable IRQs */
	  __enable_irq();
	  /* Wait until a data is received in DR register (RXNE = 1) EV7 */
	  I2C_TimeOut = I2C_TIMEOUT;
	  while ((I2C_MEMS->SR1 & 0x00040) != 0x000040)
	  {
		  if (I2C_TimeOut-- <= 0){
			  return(I2C_TIMEOUT_UserCallback());
		  }
	  }
	  /* Read the data */
      *pBuffer = I2C_MEMS->DR;

  }
  else if (numByteToRead == 2) {

    /* Set POS bit */
    I2C_MEMS->CR1 |= CR1_POS_Set;
    /* Wait until ADDR is set: EV6 */
    I2C_TimeOut = I2C_TIMEOUT;
    while ((I2C_MEMS->SR1&0x0002) != 0x0002)
    {
    	if (I2C_TimeOut-- <= 0){
    		return(I2C_TIMEOUT_UserCallback());
    	}
    }
    /* EV6_1: The acknowledge disable should be done just after EV6,
    that is after ADDR is cleared, so disable all active IRQs around ADDR clearing and
    ACK clearing */
    __disable_irq();
    /* Clear ADDR by reading SR2 register  */
    temp = I2C_MEMS->SR2;
    /* Clear ACK */
    I2C_MEMS->CR1 &= CR1_ACK_Reset;
    /*Re-enable IRQs */
    __enable_irq();
    /* Wait until BTF is set */
    I2C_TimeOut = I2C_TIMEOUT;
    while ((I2C_MEMS->SR1 & 0x00004) != 0x000004)
    {
    	if (I2C_TimeOut-- <= 0){
    		return(I2C_TIMEOUT_UserCallback());
    	}
    }
    /* Disable IRQs around STOP programming and data reading */
    __disable_irq();
    /* Program the STOP */
    I2C_GenerateSTOP(I2C_MEMS, ENABLE);
    /* Read first data */
    *pBuffer = I2C_MEMS->DR;
    /* Re-enable IRQs */
    __enable_irq();
    /**/
    pBuffer++;
    /* Read second data */
    *pBuffer = I2C_MEMS->DR;
    /* Clear POS bit */
    I2C_MEMS->CR1  &= CR1_POS_Reset;
  }


  else { //numByteToRead > 2
    // * Test on EV6 and clear it * /
    I2C_TimeOut = I2C_TIMEOUT;
    while(!I2C_CheckEvent(I2C_MEMS, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
    {
    	if (I2C_TimeOut-- <= 0){
    		return(I2C_TIMEOUT_UserCallback());
    	}
    }
    // * While there is data to be read * /
    while(numByteToRead)   {
      /* Receive bytes from first byte until byte N-3 */
    	if (numByteToRead != 3) {
        /* Poll on BTF to receive data because in polling mode we can not guarantee the
        EV7 software sequence is managed before the current byte transfer completes */
        I2C_TimeOut = I2C_TIMEOUT;
        while ((I2C_MEMS->SR1 & 0x00004) != 0x000004)
        {
            if (I2C_TimeOut-- <= 0){
                    return(I2C_TIMEOUT_UserCallback());
            }
        }
         /* Read data */
         *pBuffer = I2C_MEMS->DR;
         pBuffer++;
         /* Decrement the read bytes counter */
         numByteToRead--;
    	}

      /* it remains to read three data: data N-2, data N-1, Data N */
      if (numByteToRead == 3) {
        /* Wait until BTF is set: Data N-2 in DR and data N -1 in shift register */
        I2C_TimeOut = I2C_TIMEOUT;
        while ((I2C_MEMS->SR1 & 0x00004) != 0x000004)
        {
            if (I2C_TimeOut-- <= 0){
                    return(I2C_TIMEOUT_UserCallback());
            }
        }
        /* Clear ACK */
        I2C_MEMS->CR1 &= CR1_ACK_Reset;

        /* Disable IRQs around data reading and STOP programming */
        __disable_irq();
        /* Read Data N-2 */
        *pBuffer = I2C_MEMS->DR;
        /* Increment */
        pBuffer++;
        /* Program the STOP */
        I2C_MEMS->CR1 |= CR1_STOP_Set;
        /* Read DataN-1 */
        *pBuffer = I2C_MEMS->DR;
        /* Re-enable IRQs */
        __enable_irq();
        /* Increment */
        pBuffer++;
        /* Wait until RXNE is set (DR contains the last data) */
        I2C_TimeOut = I2C_TIMEOUT;
        while ((I2C_MEMS->SR1 & 0x00040) != 0x000040)
        {
            if (I2C_TimeOut-- <= 0){
                    return(I2C_TIMEOUT_UserCallback());
            }
        }
        /* Read DataN */
        *pBuffer = I2C_MEMS->DR;
        /* Reset the number of bytes to be read by master */
        numByteToRead = 0;
      }
    }
  }

  /* Make sure that the STOP bit is cleared by Hardware before CR1 write access */
  I2C_TimeOut = I2C_TIMEOUT;
  while ((I2C_MEMS->CR1&0x200) == 0x200)
  {
            if (I2C_TimeOut-- <= 0){
                    return(I2C_TIMEOUT_UserCallback());
            }
  }

  // * Enable Acknowledgement to be ready for another reception * /
  I2C_AcknowledgeConfig(I2C_MEMS, ENABLE);

  return 0;

}

int I2C_Master_Write(uint8_t deviceAddress, uint8_t WriteAddr, uint8_t* pBuffer, uint16_t numByteToWrite) {

  volatile int I2C_TimeOut = 0;

  /* Send STRAT condition */
  I2C_GenerateSTART(I2C_MEMS, ENABLE);

  /* Test on EV5 and clear it */
  I2C_TimeOut = I2C_TIMEOUT;
  while(!I2C_CheckEvent(I2C_MEMS, I2C_EVENT_MASTER_MODE_SELECT))
  {
    if (I2C_TimeOut-- <= 0){
            return(I2C_TIMEOUT_UserCallback());
    }
  }

  /* Send EEPROM address for write */
  I2C_Send7bitAddress(I2C_MEMS, deviceAddress, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  I2C_TimeOut = I2C_TIMEOUT;
  while(!I2C_CheckEvent(I2C_MEMS, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
    if (I2C_TimeOut-- <= 0){
            return(I2C_TIMEOUT_UserCallback());
    }
  }

  /* Send the EEPROM's internal address to write to : only one byte Address */
  I2C_SendData(I2C_MEMS, WriteAddr);

  /* Test on EV8 and clear it */
  I2C_TimeOut = I2C_TIMEOUT;
  while(!I2C_CheckEvent(I2C_MEMS, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
    if (I2C_TimeOut-- <= 0){
            return(I2C_TIMEOUT_UserCallback());
    }
  }

  while(numByteToWrite > 0) {
    /* Send the byte to be written */
    I2C_SendData(I2C_MEMS, *pBuffer);

    /* Test on EV8 and clear it */
    I2C_TimeOut = I2C_TIMEOUT;
    while(!I2C_CheckEvent(I2C_MEMS, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
      if (I2C_TimeOut-- <= 0){
            return(I2C_TIMEOUT_UserCallback());
      }
    }

    pBuffer++;
    numByteToWrite--;
  }

  /* Send STOP condition */
  I2C_GenerateSTOP(I2C_MEMS, ENABLE);

  return 0;
}

int I2C_Master_WriteByte(uint8_t deviceAddress, uint8_t WriteAddr, uint8_t data){

	  volatile int I2C_TimeOut = 0;

	  /* Send STRAT condition */
	  I2C_GenerateSTART(I2C_MEMS, ENABLE);

	  /* Test on EV5 and clear it */
	  I2C_TimeOut = I2C_TIMEOUT;
	  while(!I2C_CheckEvent(I2C_MEMS, I2C_EVENT_MASTER_MODE_SELECT))
	  {
	    if (I2C_TimeOut-- <= 0){
	            return(I2C_TIMEOUT_UserCallback());
	    }
	  }

	  /* Send EEPROM address for write */
	  I2C_Send7bitAddress(I2C_MEMS, deviceAddress, I2C_Direction_Transmitter);

	  /* Test on EV6 and clear it */
	  I2C_TimeOut = I2C_TIMEOUT;
	  while(!I2C_CheckEvent(I2C_MEMS, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	  {
	    if (I2C_TimeOut-- <= 0){
	            return(I2C_TIMEOUT_UserCallback());
	    }
	  }

	  /* Send the EEPROM's internal address to write to : only one byte Address */
	  I2C_SendData(I2C_MEMS, WriteAddr);

	  /* Test on EV8 and clear it */
	  I2C_TimeOut = I2C_TIMEOUT;
	  while(!I2C_CheckEvent(I2C_MEMS, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	  {
	    if (I2C_TimeOut-- <= 0){
	            return(I2C_TIMEOUT_UserCallback());
	    }
	  }


	    /* Send the byte to be written */
	    I2C_SendData(I2C_MEMS, data);

	    /* Test on EV8 and clear it */
	    I2C_TimeOut = I2C_TIMEOUT;
	    while(!I2C_CheckEvent(I2C_MEMS, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	    {
	      if (I2C_TimeOut-- <= 0){
	            return(I2C_TIMEOUT_UserCallback());
	      }
	    }

	  /* Send STOP condition */
	  I2C_GenerateSTOP(I2C_MEMS, ENABLE);

	  return 0;
}

//Do not use for the time being, not stable yet
uint16_t ReadReg(uint8_t DeviceAddress, uint8_t reg) {
	uint16_t value;

	I2C_TypeDef * I2C_PORT = I2C1;

	I2C_AcknowledgeConfig(I2C_PORT,ENABLE); // Enable I2C acknowledgment

	I2C_GenerateSTART(I2C_PORT,ENABLE);
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5

	I2C_Send7bitAddress(I2C_PORT, DeviceAddress,I2C_Direction_Transmitter); // Send slave address
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6

	I2C_SendData(I2C_PORT,reg); // Send register address
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8

	I2C_GenerateSTOP(I2C_PORT,ENABLE);

	I2C_GenerateSTART(I2C_PORT,ENABLE); // Send repeated START condition (aka Re-START)
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5

	I2C_Send7bitAddress(I2C_PORT,DeviceAddress,I2C_Direction_Receiver); // Send slave address for READ
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); // Wait for EV6
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)

	value = (I2C_ReceiveData(I2C_PORT) << 8); // Receive high byte
	I2C_AcknowledgeConfig(I2C_PORT,DISABLE); // Disable I2C acknowledgment
	I2C_GenerateSTOP(I2C_PORT,ENABLE); // Send STOP condition

	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
	value |= I2C_ReceiveData(I2C_PORT); // Receive low byte

	return value;
}

int I2C_TIMEOUT_UserCallback(void)
{
  /* User can add his own implementation to manage TimeOut Communication failure */
  /* Block communication and all processes */
  I2C_Master_DeInit();
  for(int i=0; i<3000; i++){__asm("nop");}
  I2C_Master_Init();
  for(int i=0; i<3000; i++){__asm("nop");}
  return -1;
}
