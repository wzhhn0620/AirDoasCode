/*
 * UsartHandleTask.c
 *
 *  Created on: Mar 7, 2024
 *      Author: wzh
 */
#include "UsartHandleTask.h"
#include "usart.h"
#include "control.h"


void Usart_Handle(void const * argument){

	while(1){

		AIRDOAS_USART1_Handle();

		STEPMotor_Set_Angle_Done();

		vTaskDelay(1);

	}

}
