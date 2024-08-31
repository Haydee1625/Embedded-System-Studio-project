#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "MPU6050.h"
#include "MPU6050_PC.h"
#include "Timer.h"
#include "Servo.h"

int16_t AX, AY, AZ, GX, GY, GZ;
float AX_Convert, AY_Convert, AZ_Convert, GX_Convert, GY_Convert, GZ_Convert;
Angle angle = {0.0};

int main(void)
{
	OLED_Init();
	Timer_Init();
	MPU6050_Init();
	Servo_Init();
	
	OLED_ShowString(1,1,"Angle:");
	OLED_ShowString(2,1,"roll:");
	OLED_ShowString(3,1,"pitch:");
	OLED_ShowString(4,1,"yaw:");
	while(1)
	{
		OLED_ShowFNum(2, 7, angle.roll, 4,1);
		OLED_ShowFNum(3, 8, angle.pitch, 3,1);
		OLED_ShowFNum(4, 7, angle.yaw, 4,1);
		//舵机角度范围0~180，angle.yaw范围-180~+180，处理后angle.yaw有效旋转范围-90~+90
		Servo_SetAngle_Yaw(-angle.yaw + 86);     
		//舵机角度范围0~180，angle.pitch范围-90~+90，处理后angle.pitch有效旋转范围-45~+90		
		//云台俯角的角度受限，为了演示俯运动只能将平台初始位置设定为仰30°
		Servo_SetAngle_Pitch(angle.pitch + 15); 
	}
}


void TIM3_IRQHandler()
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
	{
		MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
		
		MPU6050_DataConvert(&AX_Convert,&AY_Convert,&AZ_Convert,&GX_Convert,&GY_Convert,&GZ_Convert,AX, AY, AZ, GX, GY, GZ);

		Pose_Calculating(AX_Convert, AY_Convert, AZ_Convert, GX_Convert, GY_Convert, GZ_Convert,&angle); 

		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
	
}
