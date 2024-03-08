/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "tim.h"
#include "DS18B20.h"
#include "bmi160.h"

#define RXBUFFERSIZE   1 					//缓存大小
#define USART_REC_LEN  			20  		//定义�???????大接收字节数 200

extern uint8_t temp;

extern uint8_t  USART_RX_BUF[USART_REC_LEN]; 	//接收缓冲
extern uint8_t BATTERT_TESTER_RX_BUF[USART_REC_LEN];
extern uint16_t USART_RX_STA;         			//接收状�??
extern uint8_t USART_RX_START;

extern uint8_t aRxBuffer[RXBUFFERSIZE];			//HAL库USART接收Buffer

extern uint8_t USART1_RX_BUF[USART_REC_LEN];   //接收缓冲
extern uint8_t aRxBuffer1[RXBUFFERSIZE];
extern uint8_t USART1_RX_STA;//接收状�??
extern uint8_t USART_Symbol;
extern uint8_t USART1_RET_SBUF[5];
extern uint8_t USART1_RET_ALL_BUF[11];
extern uint8_t USART1_RX_CMD;
extern uint8_t USART1_RX_MOD;
#define  BUFFER_SIZE  20
extern uint8_t receive_buff[BUFFER_SIZE];
/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void AIRDOAS_USART1_Handle(void);
void Battert_Tester_Data_Convert(uint8_t *USART_RX_BUF,uint8_t  *buffer);
void USART_RET_SBUF_CREATE(uint8_t *USART_RET_BUF, uint8_t MOD, uint8_t CMD, uint8_t LEN, uint8_t PAYLOAD1, uint8_t PAYLOAD0);
void USART_RET_ALL_SBUF_CREATE(uint8_t *USART_RET_ALL_BUF, uint8_t MOD, uint8_t CMD, uint8_t LEN, uint8_t ELE_ANGLE_1, uint8_t ELE_ANGLE_2, uint8_t AZI_ANGLE_1, uint8_t AZI_ANGLE_2, uint8_t TSPEC_1, uint8_t TSPEC_2, uint8_t TELEC_1, uint8_t TELEC_2);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

