#include "stm32f10x.h"                  // Device header
#include "Delay.h"

uint8_t KEY;

void Key_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructrue;
	GPIO_InitStructrue.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructrue.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_6;
	GPIO_InitStructrue.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructrue);
}

uint8_t Key_GetNum(void)
{
	uint8_t KeyNumber = 0;
	
	if (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == 0){KeyNumber = 1;}
	if (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_2) == 0){KeyNumber = 2;}
	if (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4) == 0){KeyNumber = 3;}
	if (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6) == 0){KeyNumber = 4;}

	return KeyNumber;
}

uint8_t Key(void)//主函数while循环内调用
{
	uint8_t Temp;
	Temp=KEY;//避免Key_KeyNumber值不变，导致main函数重复执行
	KEY=0;//清零
	return Temp;
}

void Key_Loop(void)//主函数的中断函数中调用，每隔Xms判断一次，不会影响主函数进程
{
	static uint8_t NowState, LastState;
	LastState=NowState;
	NowState=Key_GetNum();
	if(LastState==1 && NowState==0){KEY=1;}//按键1松手瞬间
	if(LastState==2 && NowState==0){KEY=2;}
	if(LastState==3 && NowState==0){KEY=3;}
	if(LastState==4 && NowState==0){KEY=4;}
}
