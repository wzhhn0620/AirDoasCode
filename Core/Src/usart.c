/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "DS18B20.h"
#include "main.h"
#include "oled.h"
#include <stdio.h>
#include "control.h"

uint8_t aRxBuffer[RXBUFFERSIZE];
uint8_t temp=0;
uint8_t USART_RX_BUF[USART_REC_LEN];   //接收缓冲
uint8_t BATTERT_TESTER_RX_BUF[USART_REC_LEN];
uint16_t USART_RX_STA=0;//接收状�??
uint8_t USART_RX_START = 0;

uint8_t USART1_RX_BUF[USART_REC_LEN];   //接收缓冲
uint8_t aRxBuffer1[RXBUFFERSIZE];
uint8_t USART1_RX_STA=0;//接收状�??
uint8_t USART1_RX_MOD=0;
uint8_t USART1_RX_CMD=0;//接收状�??
uint8_t USART_Symbol=0;
uint8_t USART1_RET_SBUF[5];
uint8_t USART1_RET_ALL_BUF[11];
uint8_t receive_buff[BUFFER_SIZE];



#ifdef __GNUC__

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

PUTCHAR_PROTOTYPE
{
    //注意下面第一个参数是&husart1，因为cubemx配置了串�?????????1自动生成�?????????
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
#endif

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA1_Channel5;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA1_Channel4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 10, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	//上位机�?�信串口
	if(huart->Instance == USART1)	// 判断是由哪个串口触发的中�??????????????????
	{
		HAL_UART_Receive_IT(&huart1,aRxBuffer1,RXBUFFERSIZE);
	}
}



void USAR_UART_IDLECallback(UART_HandleTypeDef *huart)
{
    HAL_UART_DMAStop(&huart1);                                                     //停止本次DMA传输

    uint8_t data_length  = BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);   //计算接收到的数据长度

//    printf("Receive Data(length = %d): ",data_length);
//    HAL_UART_Transmit(&huart1,receive_buff,data_length,0x200);                     //测试函数：将接收到的数据打印出去
//    printf("\r\n");

    if (receive_buff[0] == 0x80) {
//		printf("0x80");
		USART1_RX_BUF[0]=receive_buff[1];
		USART1_RX_BUF[1]=receive_buff[2];
		USART1_RX_MOD=0x05;
	}else if (receive_buff[0] == 0x81 || receive_buff[0] == 0x82) {
		USART1_RX_BUF[0]=receive_buff[1];
		USART1_RX_BUF[1]=receive_buff[2];
		USART1_RX_BUF[2]=receive_buff[3];
		USART1_RX_BUF[3]=receive_buff[4];
		USART1_RX_MOD=0x04;
	}


    memset(receive_buff,0,data_length);                                            //清零接收缓冲�????
    data_length = 0;
    HAL_UART_Receive_DMA(&huart1, (uint8_t*)receive_buff, BUFFER_SIZE);                    //重启�????始DMA传输 每次255字节数据
}



void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    if(USART1 == huart1.Instance)                                   //判断是否是串�????1（！此处应写(huart->Instance == USART1)
    {
        if(RESET != __HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))   //判断是否是空闲中�????
        {
            __HAL_UART_CLEAR_IDLEFLAG(&huart1);                     //清楚空闲中断标志（否则会�????直不断进入中断）
//            printf("\r\nUART1 Idle IQR Detected\r\n");
            USAR_UART_IDLECallback(huart);                          //调用中断处理函数
        }
    }
}



void USART_RET_SBUF_CREATE(uint8_t *USART_RET_BUF, uint8_t MOD, uint8_t CMD, uint8_t LEN, uint8_t PAYLOAD1, uint8_t PAYLOAD0)
{

	USART_RET_BUF[0] = MOD;
	USART_RET_BUF[1] = CMD;
	USART_RET_BUF[2] = LEN;
	USART_RET_BUF[3] = PAYLOAD1;
	USART_RET_BUF[4] = PAYLOAD0;

}

void USART_RET_ALL_SBUF_CREATE(uint8_t *USART_RET_ALL_BUF, uint8_t MOD, uint8_t CMD, uint8_t LEN, uint8_t ELE_ANGLE_1, uint8_t ELE_ANGLE_2, uint8_t AZI_ANGLE_1, uint8_t AZI_ANGLE_2, uint8_t TSPEC_1, uint8_t TSPEC_2, uint8_t TELEC_1, uint8_t TELEC_2)
{

	USART_RET_ALL_BUF[0] = MOD;
	USART_RET_ALL_BUF[1] = CMD;
	USART_RET_ALL_BUF[2] = LEN;
	USART_RET_ALL_BUF[3] = ELE_ANGLE_1;
	USART_RET_ALL_BUF[4] = ELE_ANGLE_2;
	USART_RET_ALL_BUF[5] = AZI_ANGLE_1;
	USART_RET_ALL_BUF[6] = AZI_ANGLE_2;
	USART_RET_ALL_BUF[7] = TSPEC_1;
	USART_RET_ALL_BUF[8] = TSPEC_2;
	USART_RET_ALL_BUF[9] = TELEC_1;
	USART_RET_ALL_BUF[10] = TELEC_2;

}

void Battert_Tester_Data_Convert(uint8_t *USART_RX_BUF,uint8_t  *buffer)
{
	static uint8_t Percent_Capacity;
	static uint32_t Total_battery_voltage;
	static uint32_t Current_battery_capacity;
	static uint32_t Battery_current;
	static uint32_t Battery_time_remaining;

	Percent_Capacity = USART_RX_BUF[0];
	Total_battery_voltage = ((USART_RX_BUF[1]<<8)|USART_RX_BUF[2])*0x0a;
	Current_battery_capacity = (USART_RX_BUF[3]<<24)|(USART_RX_BUF[4]<<16)|(USART_RX_BUF[5]<<8)|USART_RX_BUF[6];
	Battery_current = (USART_RX_BUF[7]<<24)|(USART_RX_BUF[8]<<16)|(USART_RX_BUF[9]<<8)|USART_RX_BUF[10];
	Battery_time_remaining = (USART_RX_BUF[11]<<16)|(USART_RX_BUF[12]<<8)|USART_RX_BUF[13];

	sprintf((char*)buffer,"percent:%d%%,voltage:%dmV,capacity:%d,current:%dmA\r\n",Percent_Capacity,Total_battery_voltage,Current_battery_capacity,Battery_current);
}


/******************************第二代�?�信协议:串口处理函数********************************/
void AIRDOAS_USART1_Handle(void){
	//串口处理函数
	if(USART1_RX_MOD&0xfc)
	{
		if (USART1_RX_MOD==0x05) {
			USART1_RX_CMD = USART1_RX_BUF[0];
			switch (USART1_RX_CMD) {
				case 0x11:
	//						USART_SEND_SIGN = 1;
					Roll_RET = (int16_t)(Roll_x * 100);
					USART_RET_SBUF_CREATE(USART1_RET_SBUF, 0x80, USART1_RX_CMD, 0x02, (int8_t)(Roll_RET>>8), (int8_t)Roll_RET);
					HAL_UART_Transmit_DMA(&huart1,USART1_RET_SBUF,5);	//发�??
	//						while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)!=SET);		//等待发�?�完�??????????????????????????????????????
	//						USART_SEND_SIGN = 0;
					break;
				case 0x10:
	//						USART_SEND_SIGN = 1;
					Roll_RET = (int16_t)(Roll_x * 100);
					AZI_RET = 0;
					T_SPEC_RET = (int16_t)(temperature * 100);
					USART_RET_ALL_SBUF_CREATE(USART1_RET_ALL_BUF, 0x80, USART1_RX_CMD, 0x06, (int8_t)(Roll_RET>>8), (int8_t)Roll_RET, (int8_t)(AZI_RET>>8), (int8_t)AZI_RET, (int8_t)(T_SPEC_RET>>8), (int8_t)T_SPEC_RET, 0x00, 0x00);
					HAL_UART_Transmit_DMA(&huart1,USART1_RET_ALL_BUF,11);	//发�??
	//						while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)!=SET);		//等待发�?�完�??????????????????????????????????????
	//						USART_SEND_SIGN = 0;
					break;
				default:
					break;
			}
		}else if (USART1_RX_MOD==0x04) {
			len=USART1_RX_STA;//计算长度
			USART1_RX_CMD = USART1_RX_BUF[0];
			switch (USART1_RX_CMD) {
				case 0x11:
					Last_Angletarget = Angletarget;
					Last_Angleoriginal = Angleoriginal;
					USART1_ROLLX_ANGLE_U = USART1_RX_BUF[2];
					USART1_ROLLX_ANGLE_U = USART1_ROLLX_ANGLE_U<<8;
					USART1_ROLLX_ANGLE_U = USART1_ROLLX_ANGLE_U + USART1_RX_BUF[3];
					USART1_ROLLX_ANGLE = USART1_ROLLX_ANGLE_U;
					USART1_ROLLX_ANGLE = USART1_ROLLX_ANGLE;
	//						USART1_ROLLX_ANGLE = -USART1_ROLLX_ANGLE ;
	//
	//						if (USART1_ROLLX_ANGLE&0x8000) {
	////							HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
	//							USART1_ROLLX_ANGLE = 0-USART1_ROLLX_ANGLE;
	//							Angletarget = -USART1_ROLLX_ANGLE;
	////							USART_Symbol=0;
	//						}else {
	////							HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
	//							Angletarget = USART1_ROLLX_ANGLE;
	//						}
					Angletarget = ((float)USART1_ROLLX_ANGLE)/100.0;
					Angleoriginal = Angletarget;

					if (Real_Time_Ctrl == 0) {
						if (fabs(Angletarget)<0.01) {
							Stop_Ctrl = Stop_Ctrl?0:1;
						}
					}


					SET_ANGLE_COMP = 0;
					break;
				case 0x15:
	//						USART_SEND_SIGN = 1;
					USART_RET_SBUF_CREATE(USART1_RET_SBUF, 0x82, USART1_RX_CMD, 0x01, 0x01, 0x00);
					HAL_UART_Transmit_DMA(&huart1,USART1_RET_SBUF,5);	//发�??
	//						while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)!=SET);		//等待发�?�完�??????????????????????????????????????
	//						USART_SEND_SIGN = 0;
					vTaskDelay(50);
					HAL_NVIC_SystemReset();
					break;
				case 0x16:
					Angletarget = 0;
	//						USART_SEND_SIGN = 1;
					USART_RET_SBUF_CREATE(USART1_RET_SBUF, 0x81, USART1_RX_CMD, 0x01, 0x01, 0x00);
					HAL_UART_Transmit_DMA(&huart1,USART1_RET_SBUF,5);	//发�??
	//						while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)!=SET);		//等待发�?�完�??????????????????????????????????????
	//						USART_SEND_SIGN = 0;
					break;
				case 0x12:
	//						USART_SEND_SIGN = 1;
					USART_RET_SBUF_CREATE(USART1_RET_SBUF, 0x81, USART1_RX_CMD, 0x01, 0x01, 0x00);
					HAL_UART_Transmit_DMA(&huart1,USART1_RET_SBUF,5);	//发�??
	//						while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)!=SET);		//等待发�?�完�??????????????????????????????????????
	//						USART_SEND_SIGN = 0;
					break;
				default:
					break;
			}
		}


		//      		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		//      		count = USART_ROTATE_ANGLE*64/1.8+1;
		//      		HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_1);
		USART1_RX_MOD=0;
		USART1_ROLLX_ANGLE=0;
		USART1_RX_STA=0;
	}
}


/* USER CODE END 1 */
