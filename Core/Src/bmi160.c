#include "bmi160.h"
#include "oled.h"
#include "string.h"
#include "math.h"
#include "tim.h"
#include "MahonyAHRS.h"


float Pitch_y_last = 0;
float Roll_x_last = 0;
// 写指令 buffer
uint8_t I2C_Buffer_Write[16] = {0};

float Yaw_z=0;
float Pitch_y=0;
float Roll_x=0;
float Sta_Roll_x = 0;

#define sampleFreq	512.0f			// sample frequency in Hz
#define twoKpDef	(100.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.5f)	// 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

/********************************/
//volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
//volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
//volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
//volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

#define Gyro_Gr		0.0010653f		//角速度变成弧度 此参数对应陀螺2000度每秒
#define Gyro_G 		0.0610351f		//角速度变成度   此参数对应陀螺2000度每秒
#define FILTER_NUM 	20

#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f

#define S_n_sample 10

float 	AngleOffset_Rol=0,AngleOffset_Pit=0;
//float q0 = 1, q1 = 0, q2 = 0, q3 = 0;  // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0; // scaled integral error

// 定义变量
float gyr_x = 0, gyr_y = 0, gyr_z = 0,  \
                                  acc_x = 0, acc_y = 0, acc_z = 0;

float S_axs[S_n_sample] = {0}, S_ays[S_n_sample] = {0}, S_azs[S_n_sample] = {0};         //x,y轴采样队列
float S_sax=0,S_say=0,S_saz=0;
float S_gxs[S_n_sample] = {0}, S_gys[S_n_sample] = {0}, S_gzs[S_n_sample] = {0};         //x,y轴采样队列
float S_sgx=0,S_sgy=0,S_sgz=0;


//void BMI160_Write_Reg(uint8_t reg,  uint8_t val)
//{
//    I2C_Buffer_Write[0] = reg ; ;
//    I2C_Buffer_Write[1] = val;
//    HAL_I2C_Master_Transmit(&hi2c1, MBI160_ADDRESS, I2C_Buffer_Write, 2, 0xfff);
//
//}
//
//
//void BMI160_Read_Reg(uint8_t reg, uint16_t data_len)
//{
//	I2C_Buffer_Write[0] = reg;
//    I2C_Buffer_Write[1] = 0x0c;
//
//    HAL_I2C_Master_Transmit(&hi2c1, MBI160_ADDRESS, I2C_Buffer_Write, 1, 0xfff);
//
////    HAL_Delay(100);
//    memset(I2C_Buffer_Write, 0, sizeof(I2C_Buffer_Write));
//    HAL_I2C_Master_Receive(&hi2c1, MBI160_ADDRESS, I2C_Buffer_Write, data_len, 0xfff);
//}



//---------------------------------------------------------------------------------------------------
// Function declarations

/********************************/
//float invSqrt(float x);



uint8_t i2c_write_one_byte(uint8_t reg,uint8_t data)
{
  extern I2C_HandleTypeDef hi2c1;
  unsigned char W_Data=0;

  W_Data = data;
  HAL_I2C_Mem_Write(&hi2c1, MPU_WRITE, reg, I2C_MEMADD_SIZE_8BIT, &W_Data, 1, 0xfff);
//  HAL_Delay(100);

  return 0;
}

uint8_t i2c_read_one_byte(uint8_t reg)
{
  extern I2C_HandleTypeDef hi2c1;
  unsigned char R_Data=0;

  HAL_I2C_Mem_Read(&hi2c1, MPU_READ, reg, I2C_MEMADD_SIZE_8BIT, &R_Data, 1, 0xfff);
//   HAL_Delay(1);

  return R_Data;
}

// 初始化一下BMI160 若有晃动，移动，INT1 发出中断信号
void init_bmi160()
{
	uint8_t ui8Attempts = 20;
	uint8_t ui8Status = 0;

	// 重启 BMI160
	i2c_write_one_byte(AM_DEVICES_BMI160_CMD, 0xB6);

	while (ui8Status != 0x24 && ui8Attempts--)
	{
		// BMI160 陀螺仪 通用模式
		i2c_write_one_byte(AM_DEVICES_BMI160_CMD, 0x15);

		// BMI160 加速器 low power
		i2c_write_one_byte(AM_DEVICES_BMI160_CMD, 0x11);
		ui8Status = i2c_read_one_byte(AM_DEVICES_BMI160_PMU_STATUS);
//		OLED_Refresh();
//		sprintf((char*)oled_buffer,"status :%d ",ui8Status);
//		OLED_ShowString(0,0,oled_buffer,8,1);
	}
	i2c_write_one_byte(0x40, 0x0C);
	i2c_write_one_byte(0x42, 0x0C);
//	i2c_read_one_byte(AM_DEVICES_BMI160_ERR_REG, 1);
//
//
//	BMI160_Write_Reg(AM_DEVICES_BMI160_INT_MOTION_0, 0x00);
//	BMI160_Write_Reg(AM_DEVICES_BMI160_INT_MOTION_1, 0x14);
//	//BMI160_Write_Reg(AM_DEVICES_BMI160_INT_MOTION_2, 0xFF);
//	BMI160_Write_Reg(AM_DEVICES_BMI160_INT_MOTION_3, 0x02);
//
//
//	BMI160_Write_Reg(AM_DEVICES_BMI160_INT_OUT_CTRL, 0x0A);
//
//	//BMI160_Write_Reg(AM_DEVICES_BMI160_INT_MAP_2, 0x07);
//	//BMI160_Write_Reg(AM_DEVICES_BMI160_INT_MAP_1, 0x07);
//	BMI160_Write_Reg(AM_DEVICES_BMI160_INT_MAP_0, 0x07);
//
//	// INT_EN_0
//	BMI160_Write_Reg(AM_DEVICES_BMI160_INT_EN_0, 0x07);
//	BMI160_Write_Reg(AM_DEVICES_BMI160_INT_EN_1, 0x07);
//	BMI160_Write_Reg(AM_DEVICES_BMI160_INT_EN_2, 0x07);

}

//加速度三轴数据获取源码
void getAccelerometerValue(void)
{

	short x,y,z;

//	//向命令寄存器写入0x11,使加速度处于正常工作模式
	i2c_write_one_byte(0x7e,0x11);
//
//	//切换工作模式之后，延时100ms
//	HAL_Delay(2);


	//加速度 X轴///
	x =( i2c_read_one_byte(0x12) &0xff);
	x = x|(( i2c_read_one_byte(0x13) &0xff)<<8);

	acc_x = (float)x;

	//当量程为±2g时，转换为g/s的加速度换算公式
	acc_x = ((float)acc_x*9.8*2.0)/(32768.0);


	//加速度 Y轴///
	y =( i2c_read_one_byte(0x14) &0xff)  ;
	y = y|(( i2c_read_one_byte(0x15) &0xff)<<8);

	acc_y = (float)y;

	//当量程为±2g时，转换为g/s的加速度换算公式
	acc_y = ((float)acc_y*9.8*2.0)/(32768.0);


	//加速度 Z轴///
	z =( i2c_read_one_byte(0x16) &0xff)  ;
	z = z|(( i2c_read_one_byte(0x17) &0xff)<<8);

	acc_z = (float)z;

	//当量程为±2g时，转换为g/s的加速度换算公式
	acc_z = ((float)acc_z*9.8*2.0)/(32768.0);
}

//陀螺仪角速度三轴数据获取
void getGyroscopeValue(void)
{

	short x,y,z;

	//向命令寄存器写入0x15,使陀螺仪处于正常工作模式
	i2c_write_one_byte(0x7e,0x15);
//
//	//切换工作模式之后，延时100ms
//	HAL_Delay(2);

	//陀螺仪角速度 X轴///
	x =( i2c_read_one_byte(0x0c) &0xff)  ;
	x = x|(( i2c_read_one_byte(0x0d) &0xff)<<8);

	gyr_x = (float)x;

	// range为±2000°/s时，转换为角速度°/s的公式
	gyr_x = ((float)gyr_x*BMI088_GYRO_2000_SEN);


	//陀螺仪角速度 Y轴///
	y =( i2c_read_one_byte(0x0e) &0xff)  ;
	y = y|(( i2c_read_one_byte(0x0f) &0xff)<<8);

	gyr_y = (float)y;

	// range为±2000°/s时，转换为角速度°/s的公式
	gyr_y = ((float)gyr_y*BMI088_GYRO_2000_SEN);

	//陀螺仪角速度 Z轴///
	z =( i2c_read_one_byte(0x10) &0xff)  ;
	z = z|(( i2c_read_one_byte(0x11) &0xff)<<8);

	gyr_z = (float)z;

	// range为±2000°/s时，转换为角速度°/s的公式
	gyr_z = ((float)gyr_z*BMI088_GYRO_2000_SEN);
}

void AngleCalculate(float gx, float gy, float gz, float ax, float ay, float az)
{

	S_sgx = S_sgx - S_gxs[0];
	S_sgy = S_sgy - S_gys[0];
	S_sgz = S_sgz - S_gzs[0];
	S_sax = S_sax - S_axs[0];
	S_say = S_say - S_ays[0];
	S_saz = S_saz - S_azs[0];


	for(int i=1;i<S_n_sample;i++)
	{
	  S_gxs[i-1] = S_gxs[i];
	  S_gys[i-1] = S_gys[i];
	  S_gzs[i-1] = S_gzs[i];
	}

	S_gxs[S_n_sample-1] = gx;                                //x轴加速度平均值
	S_gys[S_n_sample-1] = gy;                               //y轴加速度平均值
	S_gzs[S_n_sample-1] = gz;

	S_sgx += gx;
	S_sgy += gy;
	S_sgz += gz;

	gx = S_sgx/S_n_sample;
	gy = S_sgy/S_n_sample;
	gz = S_sgz/S_n_sample;




	for(int i=1;i<S_n_sample;i++)
	{
	  S_axs[i-1] = S_axs[i];
	  S_ays[i-1] = S_ays[i];
	  S_azs[i-1] = S_azs[i];
	}

	S_axs[S_n_sample-1] = ax;                                //x轴加速度平均值
	S_ays[S_n_sample-1] = ay;                               //y轴加速度平均值
	S_azs[S_n_sample-1] = az;

	S_sax += ax;
	S_say += ay;
	S_saz += az;

	ax = S_sax/S_n_sample;
	ay = S_say/S_n_sample;
	az = S_saz/S_n_sample;




	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;

		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

//	Yaw_z = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1)* 57.2957795f;
//	Roll_x = -atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3 - AngleOffset_Rol; // roll
	Roll_x = -atan2f(2.0f * (q0 * q1 + q2 * q3), 2.0f * (q0 * q0 + q3 * q3) - 1.0f) * 57.2957795f;


//	if (Roll_x>=62) {
//		Roll_x = 1.034*Roll_x-4.138;
//	}else if (Roll_x>=32.7 && Roll_x<62) {
//		Roll_x = 1.024*Roll_x-3.481;
//	}else if (Roll_x>=12.8 && Roll_x<32.7) {
//		Roll_x = 1.005*Roll_x-2.864;
//	}else if (Roll_x>=2.8 && Roll_x<12.8) {
//		Roll_x = Roll_x-2.8;
//	}else if (Roll_x>=-7.3 && Roll_x<2.8) {
//		Roll_x = 0.9901*Roll_x-2.772;
//	}else if (Roll_x>=-27.8 && Roll_x<-7.3) {
//		Roll_x = 0.9756*Roll_x-2.878;
//	}else if (Roll_x>=-58.8 && Roll_x<-27.8) {
//		Roll_x = 0.9677*Roll_x-3.097;
//	}else if (Roll_x<-58.8) {
//		Roll_x = 0.9009*Roll_x-7.027;
//	}


	//炮筒角度校准
	if (Roll_x>=11.4) {
		Roll_x = 1.02*Roll_x-1.633;
	}else if (Roll_x>=1.8 && Roll_x<11.4) {
		Roll_x = 1.042*Roll_x-1.875;
	}else if (Roll_x>=-7.9 && Roll_x<1.8) {
		Roll_x = 1.031*Roll_x-1.856;
	}else if (Roll_x<-7.9) {
		Roll_x = 0.9744*Roll_x-2.302;
	}


////	// 抛物面镜角度校准
//	Roll_x = Roll_x+2.0f;
//
//	if (Roll_x>=10.6) {
//		Roll_x = 1.03*Roll_x-0.9138;
//	}else if (Roll_x>=0.9 && Roll_x<10.6) {
//		Roll_x = 1.031*Roll_x-0.9278;
//	}else if (Roll_x>=-9.2 && Roll_x<0.9) {
//		Roll_x = 0.9901*Roll_x-0.8911;
//	}else if (Roll_x<-9.2) {
//		Roll_x = 1.015*Roll_x-0.6599;
//	}


//	if(fabs(Roll_x)<90.0){
//	Pitch_y = -asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3 - AngleOffset_Pit; // pitch
//	}else{
//		Pitch_y = (fabs(q0* q2 - q1 * q3)/(q0* q2 - q1 * q3)) *(180.0 - fabs(asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3- AngleOffset_Pit)) ; // pitch
//	}

}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

/********************************/
//float invSqrt(float x) {
//	float halfx = 0.5f * x;
//	float y = x;
//	long i = *(long*)&y;
//	i = 0x5f3759df - (i>>1);
//	y = *(float*)&i;
//	y = y * (1.5f - (halfx * y * y));
//	return y;
//}

