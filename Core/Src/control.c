/*
 * control.c
 *
 *  Created on: Jan 13, 2024
 *      Author: wzh
 */

#include "control.h"
#include "main.h"
#include "tim.h"
#include "mpu6050.h"
#include "iwdg.h"

uint8_t Stop_Ctrl = 0;

PID motor_PID;

#define LIMIT(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x)))


void PID_Init(PID *pid,float p,float i,float d,float maxI,float maxOut)
{
	pid->kp=p;
	pid->ki=i;
	pid->kd=d;
	pid->maxIntegral=maxI;
	pid->maxOutput=maxOut;
}

//清空一个pid的历史数据
void PID_Clear(PID *pid)
{
	pid->error=0;
	pid->lastError=0;
	pid->integral=0;
	pid->output=0;
}


//单级pid计算
void PID_SingleCalc(PID *pid,float reference,float feedback)
{
	//更新数据
	pid->lastError=pid->error;
	pid->error=feedback-reference;
	//计算微分
	pid->output=(pid->error-pid->lastError)*pid->kd;
	//计算比例
	pid->output+=pid->error*pid->kp;
	//计算积分
	pid->integral+=pid->error*pid->ki;
	LIMIT(pid->integral,-pid->maxIntegral,pid->maxIntegral);//积分限幅
	pid->output+=pid->integral;
	//输出限幅
	LIMIT(pid->output,-pid->maxOutput,pid->maxOutput);
}

/**************************步进电机角度控制*********************************/
void STEPMotor_Angle_Ctrl(void){
	SECTask++;
	if (SECTask > 10) {
		if (Real_Time_Ctrl) {
			if (SET_ANGLE_COMP == 1) {
					Angle_offset = (Roll_x-Angletarget) * 0.6;
					if (fabs(Angle_offset)>=0.5) {
	//					PID_SingleCalc(&motor_PID, Angletarget, Roll_x);
	//					Angle_offset = motor_PID.output;
						if (Angle_offset>0) {
							HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
		//								HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
							count = (int)(Angle_offset*reducer*64/1.8);
						}else {
							HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
		//								HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
							count = -(int)(Angle_offset*reducer*64/1.8);
						}

						HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_1);
						SET_ANGLE_COMP = 3;
					}
			}
		}else {
			if (Stop_Ctrl == 0) {
				if (SET_ANGLE_COMP == 1) {
					Angle_offset = (Roll_x-Angletarget) * 0.6;
					if (fabs(Angle_offset)>=0.5) {
		//					PID_SingleCalc(&motor_PID, Angletarget, Roll_x);
		//					Angle_offset = motor_PID.output;
						if (Angle_offset>0) {
							HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
		//								HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
							count = (int)(Angle_offset*reducer*64/1.8);
						}else {
							HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
		//								HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
							count = -(int)(Angle_offset*reducer*64/1.8);
						}

						HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_1);
						SET_ANGLE_COMP = 3;
					}
				}
			}
		}
		HAL_IWDG_Refresh(&hiwdg);
		SECTask = 0;
	//		printf("%.2f\r\n",temperature);

	}
}

void STEPMotor_Set_Angle(void){
	if(SET_ANGLE_COMP==0){
	//		MOTORInit = 0;

		Angle_offset = Last_Angleoriginal - Angleoriginal;

		if (fabs(Angle_offset)>=0.1) {
		//		if (abs(Angle_offset)>=0.2) {
			if (Angle_offset>0) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
		//						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
				count = (int)(Angle_offset*reducer*64/1.8);
			}else {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
		//						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
				count = -(int)(Angle_offset*reducer*64/1.8);
			}
			HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_1);
			SET_ANGLE_COMP = 2;
		}else {
			count = 0;
			SET_ANGLE_COMP = 1;
			SET_ANGLE_FINISH = 1;
		}

	}
}

void STEPMotor_Set_Angle_Done(void){
	if (SET_ANGLE_FINISH == 1) {
//		HAL_Delay(100);
		if (Real_Time_Ctrl) {
			vTaskDelay(50);
			Angle_offset = Roll_x-Angletarget;
			if (fabs(Angle_offset)<1) {
	//			USART_SEND_SIGN = 1;
				USART_RET_SBUF_CREATE(USART1_RET_SBUF, 0x81, 0x11, 0x01, 0x01, 0x00);
				HAL_UART_Transmit_DMA(&huart1,USART1_RET_SBUF,5);	//发�??
	//			while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)!=SET);		//等待发�?�完�???????????????????????????????????
				SET_ANGLE_FINISH = 0;
	//			USART_SEND_SIGN = 0;
			}
		}else {
			USART_RET_SBUF_CREATE(USART1_RET_SBUF, 0x81, 0x11, 0x01, 0x01, 0x00);
			HAL_UART_Transmit_DMA(&huart1,USART1_RET_SBUF,5);	//发�??
//			while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)!=SET);		//等待发�?�完�???????????????????????????????????
			SET_ANGLE_FINISH = 0;
		}
	}
}




