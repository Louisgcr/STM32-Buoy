#include "stm32f10x.h"
#include "stm32f10x_I2C.h"

void I2C_Master_Init(void);
void I2C_Master_DeInit(void);
int I2C_Master_Read(uint8_t deviceAddr, uint8_t readAddr, uint8_t* pBuffer, uint16_t numByteToRead);
int I2C_Master_Write(uint8_t deviceAddress, uint8_t WriteAddr, uint8_t* pBuffer, uint16_t numByteToWrite);
int I2C_Master_WriteByte(uint8_t deviceAddress, uint8_t WriteAddr, uint8_t data);
uint16_t ReadReg(uint8_t DeviceAddress, uint8_t reg);
int I2C_TIMEOUT_UserCallback(void);
