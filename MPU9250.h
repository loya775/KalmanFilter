/*
 * MPU9250.h
 *
 *  Created on: 29/11/2019
 *      Author: loya_
 */

#ifndef MPU9250_H_
#define MPU9250_H_

#include "DataTypeDefinitions.h"

void Init_MPU9250(void);
uint8_t I2C_WRITE_MPU(I2C_ChannelType I2C, uint8_t SlaveAddress, uint8_t Register, uint8_t Value);

uint8_t I2C_READ_MPU(I2C_ChannelType I2C, uint8_t SlaveAddress, uint8_t Register, uint8_t* Value);

uint8_t I2C_READ_MULT_MPU(I2C_ChannelType I2C, uint8_t SlaveAddress, uint8_t Register, uint8_t* Value, uint8_t bytes);

void ReadGyro(float* GyroX, float* GyroY, float* GyroZ);

void ReadAcc(float* AccX, float* AccY, float* AccZ);

void ReadMag(float* MagX, float* MagY, float* MagZ);

void initAK8963(float* destination);

#endif /* MPU9250_H_ */
