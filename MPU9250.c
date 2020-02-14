/*
 * MPU9250.c
 *
 *  Created on: 29/11/2019
 *      Author: loya_
 */

#define DELAYI2C 40

#include "DataTypeDefinitions.h"
#include <I2C_Driver.h>
#include "GlobalFunctions.h"
#include "GPIO.h"

#define DATE_READ 				0x01

#define MPU9250_ADDRESS 0xD0
#define MPU9250_ADDRESS_W 0xD0
#define MPU9250_ADDRESS_R 0xD1
#define MAG_ADDRESS     0x0C<<1
#define MPU9250_ACCEL_CONFIG 0x1C
#define MPU9250_GYRO_CONFIG  0x1B
#define MPU9250_MAG_CONFIG 0x0A
#define PIN_CONFIG 0x37
#define ENABLE_BYPASSMODE 0x22
#define DELAYSTART 5000

#define    ACC_FULL_SCALE_2_G        0x00
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

#define    GYRO_FULL_SCALE_250_DPS    0x00
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

uint8 result[20];


uint8_t I2C_WRITE_MPU(I2C_ChannelType I2C, uint8_t SlaveAddress, uint8_t Register, uint8_t Value)
{
	/*This function initiliazes the RTC by writing in the ST in the seconds byte
		 * Before that it has to be in TX mode and it has to specify the I2C to be used as a slave.
		 * Once it has done all its processing, it checks if the oscillator is running and wait until it's stopped*/
	I2C_TX_RX_Mode(TRUE, I2C);

		/** Sends a start signal*/
	I2C_start(I2C);

		/** Writes the RTC address, in write mode, and waits for the ACK*/
	I2C_write_Byte(SlaveAddress, I2C);
	I2C_wait(I2C);
	I2C_get_ACK(I2C);

		/** Writes the seconds address and waits for the ACK*/
	I2C_write_Byte(Register, I2C);
	I2C_wait(I2C);
	I2C_get_ACK(I2C);

	I2C_write_Byte(Value, I2C);
	I2C_wait(I2C);
	I2C_get_ACK(I2C);

		/** Stop signal*/
	I2C_stop(I2C);

	return 0;
}


uint8_t I2C_READ_MPU(I2C_ChannelType I2C, uint8_t SlaveAddress, uint8_t Register, uint8_t* Value){
	/** Return variable*/
	uint8 seconds;
	//uint8 Val;
	/** Prepares the I2C to transmit*/
	I2C_TX_RX_Mode(TRUE, I2C);


	/** Sends a start signal*/
	I2C_start(I2C);

	/** Writes the RTC address, in write mode, and waits for the ACK*/
	I2C_write_Byte((SlaveAddress), I2C);
	I2C_wait(I2C);
	I2C_get_ACK(I2C);

	delay(DELAYI2C);

	/** Writes the seconds address and waits for the ACK*/
	I2C_write_Byte(Register, I2C);
	I2C_wait(I2C);
	I2C_get_ACK(I2C);
	delay(DELAYI2C);

	/** Sends a repeated start*/
	I2C_repeted_Start(I2C);

	/** Writes the RTC address, in read mode, and waits for the ACK*/
	I2C_write_Byte(((SlaveAddress) | 1), I2C);
	I2C_wait(I2C);
	I2C_get_ACK(I2C);
	delay(DELAYI2C);

	/** Prepares the I2C to receive*/
	I2C_TX_RX_Mode(FALSE, I2C);

	/** Prepares the NACK*/
	I2C_NACK(I2C);
	/** Dummy read*/
	seconds = I2C_read_Byte(I2C);
	I2C_wait(I2C);
	delay(DELAYI2C);

	/** Stop signal*/
	I2C_stop(I2C);
	/** Real read*/
	*Value = I2C_read_Byte(I2C);

	return (1);
}

uint8_t I2C_READ_MULT_MPU(I2C_ChannelType I2C, uint8_t SlaveAddress, uint8_t Register, uint8_t* Value, uint8_t bytes){
	/** Return variable*/
	uint8 seconds;
	//uint8 Val;
	uint8_t result[bytes];

	uint8_t i = 0;
	/** Prepares the I2C to transmit*/
	I2C_TX_RX_Mode(TRUE, I2C);


	/** Sends a start signal*/
	I2C_start(I2C);

	/** Writes the RTC address, in write mode, and waits for the ACK*/
	I2C_write_Byte((SlaveAddress << 1), I2C);
	I2C_wait(I2C);
	I2C_get_ACK(I2C);

	delay(DELAYI2C);

	/** Writes the seconds address and waits for the ACK*/
	I2C_write_Byte(Register, I2C);
	I2C_wait(I2C);
	I2C_get_ACK(I2C);
	delay(DELAYI2C);

	/** Sends a repeated start*/
	I2C_repeted_Start(I2C);

	/** Writes the RTC address, in read mode, and waits for the ACK*/
	I2C_write_Byte(((SlaveAddress << 1) | 1), I2C);
	I2C_wait(I2C);
	I2C_get_ACK(I2C);
	delay(DELAYI2C);

	/** Prepares the I2C to receive*/
	I2C_TX_RX_Mode(FALSE, I2C);

	/** Dummy read*/
	seconds = I2C_read_Byte(I2C);
	I2C_wait(I2C);
	delay(DELAYI2C);

	for(i = 0; i < bytes -2;i++)
	{
		result[i] = I2C_read_Byte(I2C);
		I2C_wait(I2C);
	}

	/** Prepares the NACK*/
	I2C_NACK(I2C);
	result[i++] = I2C_read_Byte(I2C);
	I2C_wait(I2C);

	/** Stop signal*/
	I2C_stop(I2C);
	/** Real read*/
	result[i++] = I2C_read_Byte(I2C);

	return (1);
}

void Init_MPU9250(void){

	I2C_WRITE_MPU(I2C_0, MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, ACC_FULL_SCALE_16_G);
	//I2C_READ_MPU(I2C_0, MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 0, 0);
	I2C_WRITE_MPU(I2C_0, MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, GYRO_FULL_SCALE_2000_DPS);
	//Enable ByPass
	I2C_WRITE_MPU(I2C_0, MPU9250_ADDRESS, PIN_CONFIG, 0x22);
	uint8_t enable;
	I2C_WRITE_MPU(I2C_0, MPU9250_ADDRESS, 0x38, 0x01);

	//I2C_READ_MPU(I2C_0, MPU9250_ADDRESS, PIN_CONFIG, &enable);

	//I2C_READ_MPU(I2C_0, MAG_ADDRESS, PIN_CONFIG, &enable);
	//I2C_READ_MPU(I2C_0, MAG_ADDRESS, 0x00, &enable);
	//I2C_WRITE_MPU(I2C_0, MAG_ADDRESS, MPU9250_MAG_CONFIG, 0x01);
}

void ReadAcc(float* AccX, float* AccY, float* AccZ)
{
	uint8_t Acc_Xout_H = 0;
	uint8_t Acc_Xout_L = 0;
	uint8_t Acc_Yout_H = 0;
	uint8_t Acc_Yout_L = 0;
	uint8_t Acc_Zout_H = 0;
	uint8_t Acc_Zout_L = 0;

	I2C_READ_MPU(I2C_0, MPU9250_ADDRESS, 0x3B, &Acc_Xout_H);
	I2C_READ_MPU(I2C_0, MPU9250_ADDRESS, 0x3C, &Acc_Xout_L);

	*AccX = (-(Acc_Xout_H<<8) | (Acc_Xout_L));
	*AccX *=9.8f/16384.0f;
	//AccX_F = 9.8*AccX/2048.0f;
	I2C_READ_MPU(I2C_0, MPU9250_ADDRESS, 0x3D, &Acc_Yout_H);
	I2C_READ_MPU(I2C_0, MPU9250_ADDRESS, 0x3E, &Acc_Yout_L);

	*AccY = (-(Acc_Yout_H<<8) | (Acc_Yout_L));
	*AccZ *=9.8f/16384.0f;
	//AccY_F = 9.8*AccY/2048.0f;
	I2C_READ_MPU(I2C_0, MPU9250_ADDRESS, 0x3F, &Acc_Zout_H);
	I2C_READ_MPU(I2C_0, MPU9250_ADDRESS, 0x40, &Acc_Zout_L);

	*AccZ = ((Acc_Zout_H<<8) | (Acc_Zout_L));
	*AccZ *=9.8f/16384.0f;
	//AccZ_F = 9.8*AccZ/2048.0f;
	return;
}

void ReadGyro(float* GyroX, float* GyroY, float* GyroZ)
{
	uint8_t Gyro_Xout_H = 0;
	uint8_t Gyro_Xout_L = 0;
	uint8_t Gyro_Yout_H = 0;
	uint8_t Gyro_Yout_L = 0;
	uint8_t Gyro_Zout_H = 0;
	uint8_t Gyro_Zout_L = 0;

	I2C_READ_MPU(I2C_0, MPU9250_ADDRESS, 0x43, &Gyro_Xout_H);
	I2C_READ_MPU(I2C_0, MPU9250_ADDRESS, 0x44, &Gyro_Xout_L);

	*GyroX = -(Gyro_Xout_H<<8) | (Gyro_Xout_L);

	I2C_READ_MPU(I2C_0, MPU9250_ADDRESS, 0x45, &Gyro_Yout_H);
	I2C_READ_MPU(I2C_0, MPU9250_ADDRESS, 0x46, &Gyro_Yout_L);

	*GyroY = -(Gyro_Yout_H<<8) | (Gyro_Yout_L);

	I2C_READ_MPU(I2C_0, MPU9250_ADDRESS, 0x47, &Gyro_Zout_H);
	I2C_READ_MPU(I2C_0, MPU9250_ADDRESS, 0x48, &Gyro_Zout_L);

	*GyroZ = (Gyro_Zout_H<<8) | (Gyro_Zout_L);
	return;
}

void ReadMag(float* MagX, float* MagY, float* MagZ)
{
	uint8_t Mag_Xout_H = 0;
	uint8_t Mag_Xout_L = 0;
	uint8_t Mag_Yout_H = 0;
	uint8_t Mag_Yout_L = 0;
	uint8_t Mag_Zout_H = 0;
	uint8_t Mag_Zout_L = 0;
	uint8_t ST;
	/**WHO AM I*/
	//I2C_READ_MPU(I2C_0, MAG_ADDRESS, 0x00, &ST);

	I2C_WRITE_MPU(I2C_0, MAG_ADDRESS, 0x0A, 0x01);
	do
	{
		I2C_READ_MPU(I2C_0, MAG_ADDRESS, 0x02, &ST);
	}while(!(ST & 0x01));

	I2C_READ_MPU(I2C_0, MAG_ADDRESS, 0x04, &Mag_Xout_H);
	I2C_READ_MPU(I2C_0, MAG_ADDRESS, 0x03, &Mag_Xout_L);

	*MagX = -(Mag_Xout_H<<8) | (Mag_Xout_L);

	I2C_READ_MPU(I2C_0, MAG_ADDRESS, 0x06, &Mag_Yout_H);
	I2C_READ_MPU(I2C_0, MAG_ADDRESS, 0x05, &Mag_Yout_L);

	*MagY = -(Mag_Yout_H<<8) | (Mag_Yout_L);

	I2C_READ_MPU(I2C_0, MAG_ADDRESS, 0x08, &Mag_Zout_H);
	I2C_READ_MPU(I2C_0, MAG_ADDRESS, 0x07, &Mag_Zout_L);

	*MagZ = (Mag_Zout_H<<8) | (Mag_Zout_L);
	return;
}

void initAK8963(float* destination)
{
// First extract the factory calibration for each magnetometer axis
uint8_t rawData[3];  // x/y/z gyro calibration data stored here
// Power down magnetometer
I2C_WRITE_MPU(I2C_0, MAG_ADDRESS, 0x0A, 0x00);
delay(2000);
I2C_WRITE_MPU(I2C_0, MAG_ADDRESS, 0x0A, 0x0F);
delay(2000);
I2C_READ_MPU(I2C_0, MAG_ADDRESS, 0x10, &rawData[0]);
delay(2000);
I2C_READ_MPU(I2C_0, MAG_ADDRESS, 0x11, &rawData[1]);
delay(2000);
I2C_READ_MPU(I2C_0, MAG_ADDRESS, 0x12, &rawData[2]);
delay(2000);
destination[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
destination[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;
destination[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f;
I2C_WRITE_MPU(I2C_0, MAG_ADDRESS, 0x0A, 0x00);
delay(2000);
// Configure the magnetometer for continuous read and highest resolution
// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
I2C_WRITE_MPU(I2C_0, MAG_ADDRESS, 0x0A, 1 << 4 | 0x06);
delay(2000);
}

