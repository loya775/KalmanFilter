/*
 * Copyright 2016-2018 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    CodeToConvert.c
 * @brief   Application entry point.
 */
#include <I2C_Driver.h>
#include <I2C_Driver.h>
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "KalmanFilter.h"
#include "fsl_i2c.h"
#include "MadgwickAHRS.h"

#include "DataTypeDefinitions.h"
#include "GPIO.h"
#include "MPU9250.h"

/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

/*
 * @brief   Application entry point.
 */
#define MSW_I2C_BAUD_RATE_200KHZ 0x16

#define MPU9250_ADDRESS 0xD1
int main(void)
{
	float AccX_F;
	float AccY_F;
	float AccZ_F;
	float GyroX_F;
	float GyroY_F;
	float GyroZ_F;
	float MagX_F;
	float MagY_F;
	float MagZ_F;
	float destination[3] = {0};
	/*LLA2NED(-103.4174,20.6067, 1584.849);
	KalmanFilter(.2, 5.1528, 0.4789, -0.6312, 0, 0);
	KalmanFilter(.2, 5.2031, 0.4741, -0.6312, 0, 0);*/
	GPIO_pinControlRegisterType MUXAlt2 = GPIO_MUX2;
	GPIO_clockGating(GPIO_B);
	GPIO_pinControlRegister(GPIO_B,BIT2,&MUXAlt2);
	GPIO_pinControlRegister(GPIO_B,BIT3,&MUXAlt2);
	/**Initializes the I2C*/
	I2C_init(I2C_0,210000,250);
	//I2C_READ_MPU(I2C_0, 0xD0, 0x75 ,14);
	Init_MPU9250();
	initAK8963(destination);
	//I2C_READ_MPU(I2C_0, MPU9250_ADDRESS, 0x75,14);
	float roll, pitch, yaw;
	while (1)
	{

		ReadGyro(&GyroX_F, &GyroY_F, &GyroZ_F);

		ReadAcc(&AccX_F, &AccY_F, &AccZ_F);

		ReadMag(&MagX_F, &MagY_F, &MagZ_F);

		MadgwickAHRSupdate((GyroZ_F*3.1416/180), (GyroY_F*3.1416/180), (GyroX_F*3.1416/180), AccZ_F, AccY_F, AccX_F, MagZ_F, MagY_F,MagX_F,&roll,&pitch,&yaw);
		LLA2NED(-103.4174,20.6067, 1584.849);
		//KalmanFilter( deltat, yaw, AccX_F, N, E, D)

		printf("%f %f %f \n", MagX_F, MagY_F, MagZ_F);
	}
    return 0;
}
