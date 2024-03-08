/*
 * Mpu6050Task.c
 *
 *  Created on: Mar 7, 2024
 *      Author: wzh
 */

#include "Mpu6050Task.h"
#include "mpu6050.h"
#include "MahonyAHRS.h"
#include "bmi160.h"
#include "usart.h"


void Mpu6050_Measure(void const * argument){

	while(1){

		MPU6050_Read_Accel();
		MPU6050_Read_Gyro();

		Roll_x_last = Roll_x;
		MahonyAHRSupdateIMU(Mpu6050_Data.Gyro_X,Mpu6050_Data.Gyro_Y,Mpu6050_Data.Gyro_Z,Mpu6050_Data.Accel_X,Mpu6050_Data.Accel_Y,Mpu6050_Data.Accel_Z);
		M_get_angle(q0, q1, q2, q3, &yaw, &pitch, &roll);
		Roll_x = (float)roll;

		if (IMU_Angle_Log) {
			printf("%.2f\r\n",Roll_x);
		}

		vTaskDelay(1);

	}
}
