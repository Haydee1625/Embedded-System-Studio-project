#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Servo.h"
#include "Key.h"
#include "Timer.h"

uint8_t KeyNum;
float Angle,Angle_Start = 90,Angle_End = 90;
uint8_t Flag;

int main(void)
{
	Timer_Init();
	Servo_Init();
	Key_Init();
	OLED_Init();
	
	OLED_ShowString(1,1,"Start:");
	OLED_ShowString(2,1,"End:");
	OLED_ShowString(3,1,"Angle:");
	
	Servo_SetAngle(Angle_Start);
	Angle = Angle_Start;
	Delay_ms(100);
	
	while (1)
	{
		OLED_ShowNum(1,7,Angle_Start,3);
		OLED_ShowNum(2,7,Angle_End,3);
		KeyNum = Key();
		if(KeyNum == 1)
		{
			Angle_Start -= 10;
			if(Angle_Start < 0) {Angle_Start += 10;}
		}
		if(KeyNum == 2)
		{
			Angle_Start += 10;
			if(Angle_Start > Angle_End){Angle_Start = Angle_End;}			

		}
		if(KeyNum == 3)
		{
			Angle_End -= 10;
			if(Angle_Start > Angle_End){Angle_End = Angle_Start;}
		}
		if(KeyNum == 4)
		{
			Angle_End += 10;
			if(Angle_End > 180) {Angle_End -= 10;}
		}
		Servo_SetAngle(Angle);
		OLED_ShowNum(3,7,Angle,3);
	}
}

void TIM3_IRQHandler(void)
{
	static uint8_t TIMCount1,TIMCount2;
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
	{
		TIMCount1++;
		TIMCount2++;
		if(Angle >= Angle_End){Flag = 1;}
		if(Angle <= Angle_Start){Flag = 0;}
		if(Flag == 1 && TIMCount1 > 10)
		{
			TIMCount1 = 0;
			Angle --;
		}
		if(Flag == 0 && TIMCount1 > 10)
		{
			TIMCount1 = 0;
			Angle ++;
		}
		
		if(TIMCount2 > 20)
		{
			TIMCount2 = 0;
			Key_Loop();
		}
		
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}
