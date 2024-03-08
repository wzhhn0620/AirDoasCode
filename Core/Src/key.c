#include "key.h"
#include "stdint-gcc.h"
#include "stdlib.h"
#include "oled.h"
#include "tim.h"


void KEY_Init(void)
{ 
 	
}



u8 KEY_Scan()
{
	static u8 key_up=1;
 
	if(key_up==0&&(KEY0==0&&KEY1==0&&KEY2==0&&KEY3==0))key_up=1;
	while(key_up)
	{ 
		 sm0_1;sm1_0;sm2_0;sm3_0;   //ɨ��
	    if(key_up&&(KEY0==1||KEY1==1||KEY2==1||KEY3==1))     //key_up Ϊ 1 
		{
				HAL_Delay(100);	//ȥ����
			    key_up=0;
				 if(KEY0==1)return 1;
				else if(KEY1==1)return 2;
				else if(KEY2==1) return 3;
				else if(KEY3==1)return 10;}
		 
		sm0_0;sm1_1;sm2_0;sm3_0;
		if(key_up&&(KEY0==1||KEY1==1||KEY2==1||KEY3==1))
	   {
				HAL_Delay(100);//ȥ����
				key_up=0;
				 if(KEY0==1)return 4;
				else if(KEY1==1)return 5;
				else if(KEY2==1)return 6;
				else if(KEY3==1)return 11;}
		 	
		sm0_0;sm1_0;sm2_1;sm3_0;
		if(key_up&&(KEY0==1||KEY1==1||KEY2==1||KEY3==1))
	   {
			HAL_Delay(100);//ȥ����
			key_up=0;
		    if(KEY0==1){return 7;}
	     	else if(KEY1==1)return 8;
		    else if(KEY2==1)return 9;
		    else if(KEY3==1)return 12;}
     
		sm0_0;sm1_0;sm2_0;sm3_1;
		if(key_up&&(KEY0==1||KEY1==1||KEY2==1||KEY3==1))
		{
	     	HAL_Delay(100);//ȥ����
			key_up=0;
				if(KEY0==1)return 13;
				else if(KEY1==1)return 0;
				else if(KEY2==1)return 15;
				else if(KEY3==1)return 16;
		}
 
		return 20;
	}
	return 20;

}








