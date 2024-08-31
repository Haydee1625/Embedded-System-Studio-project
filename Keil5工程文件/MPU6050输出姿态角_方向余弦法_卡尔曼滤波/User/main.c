#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "MPU6050.h"
#include "MPU6050_PC.h"
#include "MPU6050_KF.h"
#include "Timer.h"

int16_t AX, AY, AZ, GX, GY, GZ;
float AX_Convert, AY_Convert, AZ_Convert, GX_Convert, GY_Convert, GZ_Convert;
float pitch_acc, roll_acc;
float pitch_kf, roll_kf;

int main(void)
{
	OLED_Init();
	Timer_Init();
	MPU6050_Init();
	OLED_ShowString(1,1,"Kalman Filter:");
	OLED_ShowString(2,1,"roll:");
	OLED_ShowString(3,1,"pitch:");
	
	while(1)
	{
		OLED_ShowFNum(2, 7, roll_acc, 6,3);
		OLED_ShowFNum(3, 7, pitch_kf, 6,3);
	}
}


void TIM2_IRQHandler()
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
		MPU6050_DataConvert(&AX_Convert,&AY_Convert,&AZ_Convert,&GX_Convert,&GY_Convert,&GZ_Convert,AX, AY, AZ, GX, GY, GZ);
		ACC_calculateAngles(AX_Convert, AY_Convert, AZ_Convert,&pitch_acc, &roll_acc);
		
		Kalman_Filter_Pitch(pitch_acc,&pitch_kf,GY_Convert);
		Kalman_Filter_Roll(roll_acc,&roll_kf,GX_Convert);
		
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}
