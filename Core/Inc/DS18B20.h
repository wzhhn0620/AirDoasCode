/*
 * DS18B20.h
 *
 *  Created on: Jun 19, 2023
 *      Author: wzh
 */

#ifndef INC_DS18B20_H_
#define INC_DS18B20_H_

#include "gpio.h"
#include "tim.h"


#define DS18B20_IO_Pin GPIO_PIN_12
#define DS18B20_IO_GPIO_Port GPIOB
#define  DS18B20_DQ_IN	HAL_GPIO_ReadPin(DS18B20_IO_GPIO_Port, DS18B20_IO_Pin)
#define DS18B20_DQ_OUT_HIGH HAL_GPIO_WritePin(DS18B20_IO_GPIO_Port, DS18B20_IO_Pin, GPIO_PIN_SET)
#define DS18B20_DQ_OUT_LOW HAL_GPIO_WritePin(DS18B20_IO_GPIO_Port, DS18B20_IO_Pin, GPIO_PIN_RESET)


extern float temperature;
extern float temperature_get;
extern float Temp_output, setTem;

void HAL_Delay_us(uint16_t Delay);
uint8_t DS18B20_Init(void);
float DS18B20_Get_Temperature(void);
float DS18B20_Compute_PID(void);

#endif /* INC_DS18B20_H_ */
