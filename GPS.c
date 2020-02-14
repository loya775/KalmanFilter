/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "board.h"
#include "fsl_uart.h"
#include "GPS_IMU.h"
#include "pin_mux.h"
#include "clock_config.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* UART instance and clock */
#define DEMO_UART UART0
#define DEMO_UART_CLKSRC UART0_CLK_SRC
#define DEMO_UART_CLK_FREQ CLOCK_GetFreq(UART0_CLK_SRC)
#define DEMO_UART_IRQn UART0_RX_TX_IRQn
#define DEMO_UART_IRQHandler UART0_RX_TX_IRQHandler

#define DEMO_UART3 UART3
#define DEMO_UART_CLKSRC3 UART3_CLK_SRC
#define DEMO_UART_CLK_FREQ3 CLOCK_GetFreq(UART3_CLK_SRC)
#define DEMO_UART_IRQn3 UART3_RX_TX_IRQn
#define DEMO_UART_IRQHandler3 UART3_RX_TX_IRQHandler


/*! @brief Ring buffer size (Unit: Byte). */
#define DEMO_RING_BUFFER_SIZE 16

/*! @brief Ring buffer to save received data. */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

uint8_t g_tipString[] =
    "Uart functional API interrupt example\r\nBoard receives characters then sends them out\r\nNow please input:\r\n";

/*
  Ring buffer for data input and output, in this example, input data are saved
  to ring buffer in IRQ handler. The main function polls the ring buffer status,
  if there are new data, then send them out.
  Ring buffer full: (((rxIndex + 1) % DEMO_RING_BUFFER_SIZE) == txIndex)
  Ring buffer empty: (rxIndex == txIndex)
*/
uint8_t demoRingBuffer[DEMO_RING_BUFFER_SIZE];
volatile uint16_t txIndex; /* Index of the data to send out. */
volatile uint16_t rxIndex; /* Index of the memory to save new arrived data. */

/*******************************************************************************
 * Code
 ******************************************************************************/
uint8_t actual_data;
uint8_t GPGSA[5]="GPVTG";
uint8_t GPGGA[5]="GPGGA";
uint8_t SPEED[4];
uint8_t LATITUD[10];
uint8_t LONGITUD[11];
uint8_t ALTITUD[5];
uint8_t Flag_Speed;
uint8_t Flag_LLA;
uint8_t i = 0;
uint8_t iF = 0;
uint8_t FlagIndex=0;
uint8_t FlagIndexLLA=0;
void DEMO_UART_IRQHandler3(void)
{
    uint8_t data;

    /* If new data arrived. */
    if ((kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & UART_GetStatusFlags(DEMO_UART3))
    {
        data = UART_ReadByte(DEMO_UART3);

        /* If ring buffer is not full, add data to ring buffer. */
        if (((rxIndex + 1) % DEMO_RING_BUFFER_SIZE) != txIndex)
        {
            demoRingBuffer[rxIndex] = data;
            rxIndex++;
            rxIndex %= DEMO_RING_BUFFER_SIZE;
        }

        if(data == GPGGA[iF])
        {
        	iF+=1;
        	if(iF==5)
        	{
        			Flag_LLA=1;
        			FlagIndexLLA=(FlagIndexLLA+8);
        			iF=0;
        	}
        }else
        {
        	iF=0;
        }

        if(data == GPGSA[i])
        {
        	i+=1;
        	if(i==5)
        	{
        			Flag_Speed=1;
        			i=0;
        	}
        }else
        	i=0;

        if(data == 78 && Flag_Speed==1)
        {
        	FlagIndex = (rxIndex+1)%DEMO_RING_BUFFER_SIZE;
        	Flag_Speed=2;
        }


        if (Flag_Speed==2 && data == 75)
        {
        	SPEED[0]=demoRingBuffer[FlagIndex];
        	SPEED[1]=demoRingBuffer[(FlagIndex+2)%DEMO_RING_BUFFER_SIZE];
        	SPEED[2]=demoRingBuffer[(FlagIndex+3)%DEMO_RING_BUFFER_SIZE];
        	SPEED[3]=demoRingBuffer[(FlagIndex+4)%DEMO_RING_BUFFER_SIZE];
        }

        if(Flag_LLA)
        {
        	if(data==78)
        	{
        		for (int j=0;j<10;j++)
        			LATITUD[j]=demoRingBuffer[(FlagIndexLLA+j)%DEMO_RING_BUFFER_SIZE];
        		FlagIndexLLA = rxIndex+1;
        	}else if(data == 87)
        	{
        		for (int j=0;j<11;j++)
        			LONGITUD[j]=demoRingBuffer[(FlagIndexLLA+j)%DEMO_RING_BUFFER_SIZE];
        	     FlagIndexLLA = 0;
        		 Flag_LLA=0;
        	}
        }

    }
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

/*!
 * @brief Main function
 */
int main(void)
{
    uart_config_t config;

    BOARD_InitPins();
    BOARD_BootClockRUN();

    /*
     * config.baudRate_Bps = 115200U;
     * config.parityMode = kUART_ParityDisabled;
     * config.stopBitCount = kUART_OneStopBit;
     * config.txFifoWatermark = 0;
     * config.rxFifoWatermark = 1;
     * config.enableTx = false;
     * config.enableRx = false;
     */
   //  LoadMatriz(.5, .5, .5, .5);
    // KalmanFilter();
    UART_GetDefaultConfig(&config);
    config.baudRate_Bps = BOARD_DEBUG_UART_BAUDRATE;
    config.enableTx = true;
    config.enableRx = true;
    config.baudRate_Bps = 9600;
    UART_Init(DEMO_UART3, &config, DEMO_UART_CLK_FREQ3);
    UART_Init(DEMO_UART, &config, DEMO_UART_CLK_FREQ);
    config.baudRate_Bps = 9600;
    UART_Init(DEMO_UART3, &config, DEMO_UART_CLK_FREQ3);
    /* Send g_tipString out. */
    //UART_WriteBlocking(DEMO_UART, g_tipString, sizeof(g_tipString) / sizeof(g_tipString[0]));

    /* Enable RX interrupt. */
    UART_EnableInterrupts(DEMO_UART3, kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable);
    EnableIRQ(DEMO_UART_IRQn3);
    while (1)
    {
        /* Send data only when UART TX register is empty and ring buffer has data to send out. */
       /* while ((kUART_TxDataRegEmptyFlag & UART_GetStatusFlags(DEMO_UART)) && (rxIndex != txIndex))
        {
            UART_WriteByte(DEMO_UART, demoRingBuffer[txIndex]);

            txIndex++;
            txIndex %= DEMO_RING_BUFFER_SIZE;

        }*/
    }
}
