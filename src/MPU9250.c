//Code taken from https://github.com/kriswiner/MPU-9250/blob/master/STM32F401/
//Modified for our application
#include "MPU9250.h"
#include "MPU9250_Reg.h"
#include <stdio.h>
#include <math.h>
#include "string.h"

void calibrate_sensor(void){
	Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
	Gscale = GFS_250DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
	Mscale = MFS_16BITS; // MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution
	getAres();
	getGres();
	getMres();
	Mmode = 0x06;		 // 100Hz magnetometer data
	q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t packet_count, fifo_count;
	int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0}; // temp holder to hold bias data
	uint16_t  gyrosensitivity  = gyro_sense;   // = 131 LSB/degrees/sec
	uint16_t  accelsensitivity = acc_sense;  // = 16384 LSB/g

	//Edit these after obtaining calibration values
	magbias[0] = 302.282166;
	magbias[1] = 560.947388;
	magbias[2] = -251.464035;

	//Setting reset pin to 1
	I2C_Master_WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80);
	DelayMs(100);
	// get stable time source
	// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	I2C_Master_WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
	I2C_Master_WriteByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);

	// Configure device for bias calculation
	I2C_Master_WriteByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
	I2C_Master_WriteByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
	I2C_Master_WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
	I2C_Master_WriteByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
	I2C_Master_WriteByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
	I2C_Master_WriteByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
	DelayMs(20);

	// Configure MPU9250 gyro and accelerometer for bias calculation
	I2C_Master_WriteByte(MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
	I2C_Master_WriteByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
	I2C_Master_WriteByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	I2C_Master_WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	I2C_Master_WriteByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
	I2C_Master_WriteByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9250)
	DelayMs(40); // accumulate 40 samples in 80 milliseconds = 480 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	I2C_Master_WriteByte(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer
	I2C_Master_Read(MPU9250_ADDRESS,FIFO_COUNTH, data, 2);		// read FIFO sample count
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
	for (uint8_t i = 0; i < packet_count; i++) {
		int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
		I2C_Master_Read(MPU9250_ADDRESS, FIFO_R_W, data,12); // read data for averaging
		accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
		accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
		gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
		gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
		gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

		accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
		gyro_bias[0]  += (int32_t) gyro_temp[0];
		gyro_bias[1]  += (int32_t) gyro_temp[1];
		gyro_bias[2]  += (int32_t) gyro_temp[2];

	}

	accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	gyro_bias[0]  /= (int32_t) packet_count;
	gyro_bias[1]  /= (int32_t) packet_count;
	gyro_bias[2]  /= (int32_t) packet_count;

	// Remove gravity from the z-axis accelerometer bias calculation
	if(accel_bias[2] > 0L){
		accel_bias[2] -= (int32_t) accelsensitivity;
	}else {
		accel_bias[2] += (int32_t) accelsensitivity;
	}

	gyroBias[0] = (float) gyro_bias[0]/(float)gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
	gyroBias[1] = (float) gyro_bias[1]/(float)gyrosensitivity;
	gyroBias[2] = (float) gyro_bias[2]/(float)gyrosensitivity;

	// Output scaled accelerometer biases for manual subtraction in the main program
	accelBias[0] = (float)accel_bias[0]/(float)accelsensitivity;
	accelBias[1] = (float)accel_bias[1]/(float)accelsensitivity;
	accelBias[2] = (float)accel_bias[2]/(float)accelsensitivity;

	test = 1;
}

void init_sensor(void){
	uint8_t buf[2];
	I2C_Master_WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
	DelayMs(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

	// get stable time source
	I2C_Master_WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	I2C_Master_WriteByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);

	// Configure Gyro and Accelerometer
	// Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
	// DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
	// Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
	I2C_Master_WriteByte(MPU9250_ADDRESS, CONFIG, 0x03);

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	I2C_Master_WriteByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above

	// Set gyroscope full scale range
	// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	I2C_Master_Read(MPU9250_ADDRESS, GYRO_CONFIG, buf, 1); // get current GYRO_CONFIG register value

	buf[0] = buf[0] & ~0x02; // Clear Fchoice bits [1:0]
	buf[0] = buf[0] & ~0x18; // Clear AFS bits [4:3]
	buf[0] = buf[0] | Gscale << 3; // Set full scale range for the gyro
	I2C_Master_WriteByte(MPU9250_ADDRESS, GYRO_CONFIG, buf[0]); // Write new GYRO_CONFIG value to register

	// Set accelerometer full-scale range configuration
	I2C_Master_Read(MPU9250_ADDRESS, ACCEL_CONFIG,buf,1); // get current ACCEL_CONFIG register value
	buf[0] = buf[0] & ~0x18;  // Clear AFS bits [4:3]
	buf[0] = buf[0] | Ascale << 3; // Set full scale range for the accelerometer
	I2C_Master_WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, buf[0]); // Write new ACCEL_CONFIG register value

	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	I2C_Master_Read(MPU9250_ADDRESS, ACCEL_CONFIG2,buf,1); // get current ACCEL_CONFIG2 register value
	buf[0] = buf[0] & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	buf[0] = buf[0] | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	I2C_Master_WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG2, buf[0]); // Write new ACCEL_CONFIG2 register value

	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
	// but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
	// can join the I2C bus and all can be controlled by the Arduino as master
	I2C_Master_WriteByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
	I2C_Master_WriteByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt

	I2C_Master_WriteByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
	DelayMs(10);
	I2C_Master_WriteByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
	DelayMs(10);

	//Initalise Magnetometer
	uint8_t rawData[3];
	I2C_Master_Read(AK8963_ADDRESS, AK8963_ASAX, rawData,3 );  // Read the x-, y-, and z-axis calibration values
	magCalibration[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
	magCalibration[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;
	magCalibration[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f;

	I2C_Master_WriteByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
	DelayMs(10);
	// Configure the magnetometer for continuous read and highest resolution
	// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
	I2C_Master_WriteByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
	DelayMs(10);
}


void read_acc(void){
	uint8_t buf[6];
	memset(buf,'\0', sizeof(buf));
	I2C_Master_Read(MPU9250_ADDRESS , ACCEL_XOUT_H, buf ,6);

	int16_t temp = 0;
	temp = ((int16_t)(buf[0]<<8) | buf[1]);
	acc_x = (float)temp*aRes - accelBias[0];
	temp = ((int16_t)(buf[2]<<8)| buf[3]);
	acc_y = (float)temp*aRes - accelBias[1];
	temp = ((int16_t)(buf[4]<<8)| buf[5]);
	acc_z = (float)temp*aRes - accelBias[2];

	/*
	//Debugging lines
	uint8_t check[100];
	memset(check,'\0', sizeof(check));
	sprintf(check, "acc_x: %f acc_y: %f acc_z: %f \r\n", acc_x, acc_y, acc_z);
	SendData(check, sizeof(check));
	*/
	return;
}

void read_mag(void){

	uint8_t buf[7];
	memset(buf,'\0', sizeof(buf));
	I2C_Master_Read(AK8963_ADDRESS, AK8963_ST1, buf ,1);
	if(buf[0] & 0x01){
		I2C_Master_Read(AK8963_ADDRESS, AK8963_XOUT_L, buf ,7);
		if(!(buf[6] & 0x08)){ // magnetic sensor not overflow
			int16_t temp;
			temp= ((int16_t)(buf[1] << 8) | buf[0]);  // Turn the MSB and LSB into a signed 16-bit value
			mag_x = (float)temp*mRes*magCalibration[0]- magbias[0];
			temp = ((int16_t)(buf[3] << 8) | buf[2]);  // Data stored as little Endian
			mag_y = (float)temp*mRes*magCalibration[1] - magbias[1];
			temp = ((int16_t)(buf[5] << 8) | buf[4]);
			mag_z = (float)temp*mRes*magCalibration[2] - magbias[2];
			/*
			Debugging Lines
			uint8_t check[100];
			memset(check,'\0', sizeof(check));
			sprintf(check, "mx:%f, my:%f,mz:%f \r\n", mag_x,mag_y,mag_z);
			SendData(check, sizeof(check));
			*/
		}
	}

	return;

}

void read_gyro(void){
	uint8_t buf[6];
	memset(buf,'\0', sizeof(buf));

	I2C_Master_Read(MPU9250_ADDRESS , GYRO_XOUT_H, buf ,6);

	int16_t temp = 0;
	temp = ((int16_t)buf[0]<<8 | buf[1]);
	gyro_x = (float)temp*gRes - gyroBias[0];
	temp = ((int16_t)buf[2]<<8 | buf[3]);
	gyro_y = (float)temp*gRes - gyroBias[1];
	temp = ((int16_t)buf[4]<<8 | buf[5]);
	gyro_z = (float)temp*gRes - gyroBias[2];

	/*
	//Debugging lines
	uint8_t check[100];
	memset(check,'\0', sizeof(check));
	sprintf(check, "gx:%f, gy:%f,gz:%f \r\n", gyro_x,gyro_y,gyro_z);
	SendData(check, sizeof(check));
	*/
	return;
}

void run_mag_calibration(){
	read_mag();

	if(mag_x >magMaxX){
		magMaxX = mag_x;
	}
	if(mag_y >magMaxY){
		magMaxY = mag_y;
	}
	if(mag_z >magMaxZ){
		magMaxZ = mag_z;
	}

	if(mag_x < magMinX){
		magMinX = mag_x;
	}
	if(mag_y < magMinY){
		magMinY = mag_y;
	}
	if(mag_z < magMinZ){
		magMinZ = mag_z;
	}

	//Debugging lines
	uint8_t check[100];
	memset(check,'\0', sizeof(check));
	sprintf(check, "mx:%f, my:%f,mz:%f \r\n", (magMaxX + magMinX)/2,(magMaxY + magMinY)/2,(magMaxZ + magMinZ)/2);
	SendData(check, sizeof(check));
}

//---------------------Sensitivity Setting ----------------------------//
void getMres() {
	switch (Mscale)
	{
	// Possible magnetometer scales (and their register bit settings) are:
	// 14 bit resolution (0) and 16 bit resolution (1)
	case MFS_14BITS:
		mRes = 10.0*4912.0/8190.0; // Proper scale to return milliGauss
		break;
	case MFS_16BITS:
		mRes = 10.0*4912.0/32760.0; // Proper scale to return milliGauss
		break;
	}
}


void getGres() {
	switch (Gscale)
	{
	// Possible gyro scales (and their register bit settings) are:
	// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
	// Here's a bit of an algorithm to calculate DPS/(ADC tick) based on that 2-bit value:
	case GFS_250DPS:
		gRes = 250.0/32768.0;
		break;
	case GFS_500DPS:
		gRes = 500.0/32768.0;
		break;
	case GFS_1000DPS:
		gRes = 1000.0/32768.0;
		break;
	case GFS_2000DPS:
		gRes = 2000.0/32768.0;
		break;
	}
}


void getAres() {
	switch (Ascale)
	{
	// Possible accelerometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
	// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
	case AFS_2G:
		aRes = 2.0/32768.0;
		break;
	case AFS_4G:
		aRes = 4.0/32768.0;
		break;
	case AFS_8G:
		aRes = 8.0/32768.0;
		break;
	case AFS_16G:
		aRes = 16.0/32768.0;
		break;
	}
}

//---------------------Madgwick Algorithm----------------------------//
//Fast inverse square with lower error
//https://pizer.wordpress.com/2008/10/12/fast-inverse-square-root/
float invSqrt(float x){
	uint32_t i = 0x5F1F1412 - (*(uint32_t*)&x >> 1);
	float tmp = *(float*)&i;
	return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
}

void madgwick(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz){
	//GYRO in rads
	//Mag and ACC will be normalized

	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= betaDef * s0;
		qDot2 -= betaDef * s1;
		qDot3 -= betaDef * s2;
		qDot4 -= betaDef * s3;

	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

}


uint8_t checksum(uint8_t f8, uint8_t* data, size_t numBytes){

    unsigned long sum1, sum2;
    size_t part;

    sum1 = f8 & 0xf;
    sum2 = (f8 >> 4) & 0xf;
    while (numBytes) {
        part = numBytes > 5803 ? 5803 : numBytes;
        numBytes -= part;
        do {
            sum2 += sum1 += *data++;
        } while (--part);
        sum1 %= 15;
        sum2 %= 15;
    }
    return (sum2 << 4) + sum1;
}

void AHRS_Send(void){


	madgwick(-1.0*gyro_y*PI/180.0,-1.0*gyro_x*PI/180.0, -1.0*gyro_z*PI/180.0,-1.0*acc_y,-1.*acc_x,acc_z,mag_x,mag_y, mag_z);

	//Converts Float to bytes to send out via serial
	Convertq0.q = q0;
	Convertq1.q = q1;
	Convertq2.q = q2;
	Convertq3.q = q3;

	//Packs bytes into buffer
	uint8_t buf[21];
	memset(buf, '\0' , sizeof(buf));

	buf[0] = 255;	//Start
	buf[1] = 255;	//Start
	buf[2] = 18;	//Number of Bytes more
	buf[3] = 2;		//Instruction, not in use currently

	buf[4] = Convertq0.bytes[0];
	buf[5] = Convertq0.bytes[1];
	buf[6] = Convertq0.bytes[2];
	buf[7] = Convertq0.bytes[3];

	buf[8] = Convertq1.bytes[0];
	buf[9] = Convertq1.bytes[1];
	buf[10] = Convertq1.bytes[2];
	buf[11] = Convertq1.bytes[3];

	buf[12] = Convertq2.bytes[0];
	buf[13] = Convertq2.bytes[1];
	buf[14] = Convertq2.bytes[2];
	buf[15] = Convertq2.bytes[3];

	buf[16] = Convertq3.bytes[0];
	buf[17] = Convertq3.bytes[1];
	buf[18] = Convertq3.bytes[2];
	buf[19] = Convertq3.bytes[3];

	buf[20] = checksum(1,(buf+4),16);
	SendData(buf, sizeof(buf));
	/*
	uint8_t buff[500];
	memset(buff, '\0' , sizeof(buff));
	sprintf(buff, "gx:%f gy: %f gz:%f ax:%f ay: %f az:%f mx:%f my: %f mz:%f \r\n", gyro_x, gyro_y, gyro_z,acc_x,acc_y,acc_z,mag_x,mag_y,mag_z);
	SendData(buff, sizeof(buff));
	float yaw, pitch, roll;
	yaw   = atan2f(2.0 * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3);
	pitch = -asinf(2.0f * (q1*q3 - q0*q2));
	roll  = atan2f(2.0 * (q0*q1 + q2*q3), q0*q0 - q1*q1 - q2*q2 + q3*q3);
	pitch *= 180.0f / PI;
	yaw   *= 180.0f / PI;
	roll  *= 180.0f / PI;
	uint8_t buff[500];
	memset(buff, '\0' , sizeof(buff));
	sprintf(buff, "roll: %f pitch: %f yaw:%f \r\n", roll, pitch, yaw);
	SendData(buff, sizeof(buff));
	*/


	/*
			  (__)        		QUACK
			  (oo)		QUACK
	   /-------\/    			  QUACK
	  / |     ||
	 *  ||----||
		^^    ^^
	*/
}


