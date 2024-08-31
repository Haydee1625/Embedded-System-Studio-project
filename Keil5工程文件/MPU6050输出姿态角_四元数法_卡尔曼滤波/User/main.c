#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "MPU6050.h"
#include "MPU6050_PC.h"
#include "MPU6050_KF.h"
#include "Timer.h"

uint8_t KeyNum;
int16_t AX, AY, AZ, GX, GY, GZ;
float AX_Convert, AY_Convert, AZ_Convert, GX_Convert, GY_Convert, GZ_Convert;
Angle angle = {0.0};
Angle_KF angle_kf = {0.0};

int main(void)
{
	OLED_Init();
	Timer_Init();
	MPU6050_Init();
//	OLED_ShowString(1,1,"Angle:");
//	OLED_ShowString(2,1,"roll:");
//	OLED_ShowString(3,1,"pitch:");
//	OLED_ShowString(4,1,"yaw:");
	while(1)
	{
		OLED_ShowFNum(2, 7, angle.pitch, 6,3);
		OLED_ShowFNum(3, 7, angle_kf.pitch_kf, 6,3);
		OLED_ShowFNum(4, 7, angle.yaw, 4,1);
	}
}


void TIM2_IRQHandler()
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
		MPU6050_DataConvert(&AX_Convert,&AY_Convert,&AZ_Convert,&GX_Convert,&GY_Convert,&GZ_Convert,AX, AY, AZ, GX, GY, GZ);
		Pose_Calculating(AX_Convert, AY_Convert, AZ_Convert, GX_Convert, GY_Convert, GZ_Convert,&angle); 

		Kalman_Filter_Pitch(&angle,&angle_kf,GY_Convert);

		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
	
}
