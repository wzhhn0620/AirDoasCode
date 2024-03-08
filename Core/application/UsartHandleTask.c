/*
 * UsartHandleTask.c
 *
 *  Created on: Mar 7, 2024
 *      Author: wzh
 */
#include "UsartHandleTask.h"
#include "usart.h"


void Usart_Handle(void const * argument){

	while(1){

		AIRDOAS_USART1_Handle();

		vTaskDelay(1);

	}

}
