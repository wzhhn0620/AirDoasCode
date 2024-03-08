#ifndef __KEY_H
#define __KEY_H	 
#include "gpio.h"

#define u8 uint8_t
#define u32 uint32_t
#define u16 uint16_t
//#define Beep_out   PFout(8)

#define sm0_0 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET)   //sao_miao
#define sm0_1 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET)
#define sm1_0 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET)
#define sm1_1 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET)
#define sm2_0 HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,GPIO_PIN_RESET)
#define sm2_1 HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,GPIO_PIN_SET)
#define sm3_0 HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_RESET)
#define sm3_1 HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_SET)
#define KEY3  HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_2)   //????0
#define KEY2  HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)   //????1
#define KEY1  HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3)   //????2 
#define KEY0  HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_5)   //????
                                        
#define KEY0_PRES 	1	 
#define KEY1_PRES	2
#define KEY2_PRES	3 
#define KEY3_PRES   4
#define KEY4_PRES  	5 
#define KEY5_PRES   6
#define KEY6_PRES   7
#define KEY7_PRES   8
#define KEY8_PRES   9
#define KEY9_PRES    10
#define KEY10_PRES   11 
#define KEY11_PRES   12 
#define KEY12_PRES   13 
#define KEY13_PRES   14 
#define KEY14_PRES   15
#define KEY15_PRES   16

void KEY_Init(void);
u8 KEY_Scan(void);

#endif

