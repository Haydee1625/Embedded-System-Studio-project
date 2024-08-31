#include "stm32f10x.h"                  // Device header
#include "math.h"
#include "PID.h"

#define PID_LIMIT_MIN -10000		//PID输出最低值
#define PID_LIMIT_MAX 10000	        //PID输出最大值


/************************采样周期未知且不变************************************/

void PID_Init(PID_LocTypeDef *PID)
{
	PID->Kp = 0;                       //比例系数Proportional
    PID->Ki = 0;                       //积分系数Integral
    PID->Kd = 0;                       //微分系数Derivative
//	float Ti;                       //积分时间常数
//  float Td;                       //微分时间常数
//	float dt;			       		//采样周期
    PID->ek = 0.0f;                       //当前误差
    PID->ek_prev = 0.0f;                      //前一次误差e(k-1)
    PID->ek_prev_prev = 0.0f;                 //再前一次误差e(k-2)
    PID->location_sum = 0.0f;             //累计积分位置
    PID->out = 0.0f;		
}

//位置式PID
//pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
//setvalue : 设置值（期望值）
//actualvalue: 实际值
//由于全量输出，每次输出均与过去状态有关，计算时要对ek累加，计算量大

float PID_location(float setvalue, float actualvalue, PID_LocTypeDef *PID)
{ 
	PID->ek = setvalue-actualvalue;
	PID->location_sum += PID->ek;                         //计算累计误差值
	
	//积分限幅
	if((PID->Ki != 0)&&(PID->location_sum > (PID_LIMIT_MAX/PID->Ki))) PID->location_sum = PID_LIMIT_MAX/PID->Ki;
	if((PID->Ki != 0)&&(PID->location_sum < (PID_LIMIT_MIN/PID->Ki))) PID->location_sum = PID_LIMIT_MIN/PID->Ki;
	
    PID->out = PID->Kp*PID->ek + (PID->Ki*PID->location_sum) + PID->Kd*(PID->ek-PID->ek_prev);
    PID->ek_prev = PID->ek;
	
	//PID->out限幅
	if(PID->out < PID_LIMIT_MIN)	PID->out = PID_LIMIT_MIN;
	if(PID->out > PID_LIMIT_MAX)	PID->out = PID_LIMIT_MAX;
	
	return PID->out;
}


//增量式PID
//pidout += Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
//setvalue : 设置值（期望值）
//actualvalue: 实际值

float PID_increment(float setvalue, float actualvalue, PID_LocTypeDef *PID)
{                                
	PID->ek = setvalue - actualvalue;
    PID->out += PID->Kp*(PID->ek - PID->ek_prev) + PID->Ki*PID->ek + PID->Kd * (PID->ek -2*PID->ek_prev+PID->ek_prev_prev);
//	PID->out+=PID->kp*PID->ek-PID->ki*PID->ek1+PID->kd*PID->ek2;
    PID->ek_prev_prev = PID->ek_prev;
    PID->ek_prev = PID->ek;  
	
	//PID->out限幅	
	if(PID->out < PID_LIMIT_MIN)	PID->out = PID_LIMIT_MIN;
	if(PID->out > PID_LIMIT_MAX)	PID->out = PID_LIMIT_MAX;
	
	return PID->out;
}
