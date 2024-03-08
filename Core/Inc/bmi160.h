#ifndef __BMI160_H__
#define __BMI160_H__
// 0x68 << 1 = 0xD0

#include "gpio.h"
#include "i2c.h"

#define MBI160_ADDRESS 0xD2
#define MPU_READ    0XD2
#define MPU_WRITE   0XD2

#define AM_DEVICES_BMI160_CMD 0x7e
#define AM_DEVICES_BMI160_INT_MOTION_3 0x62
#define AM_DEVICES_BMI160_INT_MOTION_2 0x61
#define AM_DEVICES_BMI160_INT_MOTION_1 0x60
#define AM_DEVICES_BMI160_INT_MOTION_0 0x5F
#define AM_DEVICES_BMI160_INT_MAP_1 0x56
#define AM_DEVICES_BMI160_INT_MAP_0 0x55
#define AM_DEVICES_BMI160_INT_LATCH 0x54
#define AM_DEVICES_BMI160_INT_OUT_CTRL 0x53
#define AM_DEVICES_BMI160_INT_EN_2 0x52
#define AM_DEVICES_BMI160_INT_EN_1 0x51
#define AM_DEVICES_BMI160_INT_EN_0 0x50
#define AM_DEVICES_BMI160_PMU_STATUS 0x03
#define AM_DEVICES_BMI160_ERR_REG 0x02

#define Pi 3.1415926



// 写指令 buffer
extern uint8_t I2C_Buffer_Write[16];

extern float Yaw_z;
extern float Pitch_y;
extern float Roll_x;
extern float Pitch_y_last;
extern float Roll_x_last;
extern float Sta_Roll_x;

// 定义变量
extern float gyr_x, gyr_y, gyr_z,  \
                                  acc_x , acc_y, acc_z;

uint8_t i2c_write_one_byte(uint8_t reg,uint8_t data) ;
uint8_t i2c_read_one_byte(uint8_t reg);
void init_bmi160();
void getAccelerometerValue(void);
void getGyroscopeValue(void);
void AngleCalculate(float gx, float gy, float gz, float ax, float ay, float az);
void K_AngleCalculate(float gx, float gy, float gz, float ax, float ay, float az);

#endif
