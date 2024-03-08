#ifndef FIRFILTER_H
#define FIRFILTER_H
#include <stdint.h>

#define FIR_FILTER_LENGTH 16 //定义了滤波器的长度

typedef struct {
	float buf[FIR_FILTER_LENGTH]; //定义了滤波器存储数组
	uint8_t bufIndex;
	float out;	//滤波器当前的输出
}FIRFilter;

void FIRFilter_Init(FIRFilter *fir); //初始化函数，定义了一个指针指向结构体
float FIRFilter_Update(FIRFilter *fir,float inp);//滤波器主函数，输入为结构体指针与当前输入采样值

#endif
