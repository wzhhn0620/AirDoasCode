/*
 * control.h
 *
 *  Created on: Jan 13, 2024
 *      Author: wzh
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

typedef struct _PID
{
	float kp,ki,kd;
	float error,lastError;//误差、上次误差
	float integral,maxIntegral;//积分、积分限幅
	float output,maxOutput;//输出、输出限幅
}PID;

extern PID motor_PID;

void PID_Init(PID *pid,float p,float i,float d,float maxI,float maxOut);
void PID_Clear(PID *pid);
void PID_SingleCalc(PID *pid,float reference,float feedback);
void STEPMotor_Angle_Ctrl(void);
void STEPMotor_Set_Angle(void);

#endif /* INC_CONTROL_H_ */