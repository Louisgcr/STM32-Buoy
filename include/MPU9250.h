#ifndef MPU9250_H_
#define MPU9250_H_

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "diag/Trace.h"

#include "delay.h"
#include "I2C_Master.h"
#include "MPU9250_Reg.h"
#include "stm32f10x.h"
#include "usart.h"

#define gyro_sense  131
#define acc_sense  16384

uint8_t Mmode;        // Either 8 Hz 0x02) or 100 Hz (0x06) magnetometer data ODR
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output

int16_t tempCount;   // Stores the real internal chip temperature in degrees Celsius
float temperature;
float SelfTest[6];

float acc_x, acc_y, acc_z;
float mag_x, mag_y, mag_z;
float gyro_x, gyro_y, gyro_z;


//------------------Calibrate & Initalise Sensor--------------------
float gyroBias[3], accelBias[3] ; // Bias corrections for gyro and accelerometer
float magCalibration[3], magbias[3];  // Factory mag calibration and mag bias
float magMaxX, magMinX, magMaxY, magMinY, magMaxZ, magMinZ;
void calibrate_sensor(void);
void init_sensor(void);

/*Reading */
void read_acc(void);
void read_mag(void);
void read_gyro(void);
void run_mag_calibration(void);

//Sensitivity Configuration, update this when accelrometer setting is changed
uint8_t Ascale; // AFS_2G, AFS_4G, AFS_8G, AFS_16G
uint8_t Gscale; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
uint8_t Mscale; // MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution

//Get sensitivity
void getAres();
void getGres();
void getMres();


//-------------------Madgwick Algorithm------------------------
#define PI						3.14159265359
#define sampleFreq	50.0f
#define betaDef		1.8f		// 2 * proportional gain

char test;
volatile float beta;
volatile float q0,q1,q2,q3;

//Converts float to 4 bytes
union Quarts{
  float q;
  unsigned char bytes[4];
};
union Quarts Convertq0,Convertq1,Convertq2,Convertq3;

void madgwick(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
float invSqrt(float x);
uint8_t checksum(uint8_t f8, uint8_t* data, size_t numBytes);
void AHRS_Send(void);

#endif
