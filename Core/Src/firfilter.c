/*
 * firfilter.c
 *
 *  Created on: Sep 4, 2022
 *      Author: wzh
 */

#include "firfilter.h"

static float FIR_IMPULSE_RESPONSE[FIR_FILTER_LENGTH]={};

void FIRFilter_Init(FIRFilter *fir){
	/*清楚buf中的变量*/
	for(uint8_t n=0; n < FIR_FILTER_LENGTH; n++){
		fir->buf[n] = 0.0f;
	}
	/*重置buffer_index*/
	fir->bufIndex = 0;
	/*清除buf输出*/
	fir->out = 0.0f;
}

//返回值为最新的滤波器输出
float FIRFilter_Update(FIRFilter *fir, float inp){
	/*存储最新的采样值到buf*/
	fir->buf[fir->bufIndex] = inp;

	/*如有必要，增加缓冲区索引并环绕*/
	fir->bufIndex++;
	if(fir->bufIndex == FIR_FILTER_LENGTH){
		fir->bufIndex = 0;
	}

	/*计算新的输出样本（通过卷积） */
	fir->out = 0.0f;

	uint8_t sumIndex = fir->bufIndex;

	for(uint8_t n = 0; n<FIR_FILTER_LENGTH; n++){
		/*如有必要，减少索引和换行*/
		if (sumIndex>0) {
			sumIndex--;
		} else {
			sumIndex = FIR_FILTER_LENGTH - 1;
		}
		/*将脉冲响应与移位输入样本相乘，并添加到输出*/
		fir->out += FIR_IMPULSE_RESPONSE[n]*fir->buf[sumIndex];

	}
	/*输出fir结果*/
	return fir->out;

}
