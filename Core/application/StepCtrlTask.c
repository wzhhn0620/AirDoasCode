/*
 * StepCtrlTask.c
 *
 *  Created on: Mar 7, 2024
 *      Author: wzh
 */

#include "StepCtrlTask.h"
#include "control.h"
#include "main.h"
#include "tim.h"
#include "mpu6050.h"
#include "iwdg.h"

void StepMotor_Ctrl(void const * argument){

	while(1){

		STEPMotor_Set_Angle();

		STEPMotor_Angle_Ctrl();

		vTaskDelay(1);

	}
}
