#ifndef __PID_H__
#define __PID_H__
 
typedef struct
{
    float Kp;                       //比例系数Proportional
    float Ki;                       //积分系数Integral
    float Kd;                       //微分系数Derivative
//	float Ti;                       //积分时间常数
//  float Td;                       //微分时间常数
//	float dt;			       		//采样周期
    float ek;                       //当前误差
    float ek_prev;                      //前一次误差e(k-1)
    float ek_prev_prev;                 //再前一次误差e(k-2)
    float location_sum;             //累计积分位置
    float out;			   		  //PID输出值
}PID_LocTypeDef;

void PID_Init(PID_LocTypeDef *PID);
float PID_location(float setvalue, float actualvalue, PID_LocTypeDef *PID);
float PID_increment(float setvalue, float actualvalue, PID_LocTypeDef *PID);

#endif
