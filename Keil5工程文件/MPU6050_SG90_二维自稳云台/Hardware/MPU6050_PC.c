#include "stm32f10x.h"                  // Device header
#include "math.h"
#include "MPU6050_PC.h"

#define pi             3.1415926f

float q0 = 1.0,q1 = 0.0,q2 = 0.0,q3 = 0.0;
float Kp = 0.4,Ki = 0.0005;
float exInt = 0.0, eyInt = 0.0, ezInt = 0.0;
volatile float roll = 0.0,pitch = 0.0,yaw = 0.0;
float halfT = 0.01;//分频后采样频率100Hz

void MPU6050_DataConvert(float * AX_Convert,float *  AY_Convert,float *  AZ_Convert,
	float * GX_Convert,float *  GY_Convert,float *  GZ_Convert,int16_t AX,int16_t AY,
		int16_t AZ,int16_t GX,int16_t GY,int16_t GZ)
{
	*AX_Convert = (float)AX;
	*AY_Convert = (float)AY;
	*AZ_Convert = (float)AZ;
	*GX_Convert = (float)GX;
	*GY_Convert = (float)GY;
	*GZ_Convert = (float)GZ;
	
	*AX_Convert = *AX_Convert / 32768 * 16;
	*AY_Convert = *AY_Convert / 32768 * 16;
	*AZ_Convert = *AZ_Convert / 32768 * 16;
	*GX_Convert = *GX_Convert / 32768 * 2000 * 0.0174532925;
	*GY_Convert = *GY_Convert / 32768 * 2000 * 0.0174532925;
	*GZ_Convert = *GZ_Convert / 32768 * 2000 * 0.0174532925;
}

void Pose_Calculating(float AX_Convert, float AY_Convert, float AZ_Convert, float GX_Convert, float GY_Convert, float GZ_Convert,Angle* angle) 
{ 
	float Norm_A,Norm_Q; 
    float vx, vy, vz; 
	float ex, ey, ez;   
	
	Norm_A = sqrt(AX_Convert * AX_Convert + AY_Convert * AY_Convert + AZ_Convert * AZ_Convert);       

	AX_Convert = AX_Convert / Norm_A; 
	AY_Convert = AY_Convert / Norm_A; 
	AZ_Convert = AZ_Convert / Norm_A;  
	
//	estimated direction of gravity 
	vx = 2*(q1*q3 - q0*q2);
	vy = 2*(q0*q1 + q2*q3); 
	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3; 
	
//叉乘求姿态误差
	ex = (AY_Convert * vz - AZ_Convert * vy);
	ey = (AZ_Convert * vx - AX_Convert * vz);
	ez = (AX_Convert * vy - AY_Convert * vx);

//对误差进行PI计算，补偿角速度
    exInt = exInt + ex * Ki;
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;
 
    GX_Convert = GX_Convert + Kp*ex + exInt;
    GY_Convert = GY_Convert + Kp*ey + eyInt;
    GZ_Convert = GZ_Convert + Kp*ez + ezInt;
 
//按照四元数微分公式进行四元数更新
    q0 = q0 + (-q1*GX_Convert - q2*GY_Convert - q3*GZ_Convert)*halfT;
    q1 = q1 + (q0*GX_Convert - q3*GY_Convert + q2*GZ_Convert)*halfT;
    q2 = q2 + (q3*GX_Convert + q0*GY_Convert - q1*GZ_Convert)*halfT;
    q3 = q3 + (-q2*GX_Convert + q1*GY_Convert + q0*GZ_Convert)*halfT;
 
    Norm_Q = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0/Norm_Q;
    q1 = q1/Norm_Q;
    q2 = q2/Norm_Q;
    q3 = q3/Norm_Q;
 
   angle->roll =  atan2f(2*q2*q3 + 2*q0*q1,q0*q0 + q3*q3 - q1*q1 - q2*q2) * 180 / pi;
//	angle->roll =  atan2f(-2*q2*q3 - 2*q0*q1,q0*q0 + q3*q3 - q1*q1 - q2*q2) * 180 / pi;
   angle->pitch =  asinf(-2*q1*q3 + 2*q0*q2)* 180 / pi;
//	angle->pitch =  asinf(2*q1*q3 + 2*q0*q2)* 180 / pi;
	
//	if(GZ_Convert*180/pi > 0.5f || GZ_Convert*180/pi < -0.5f)	
//	{
		//angle->yaw +=  GZ_Convert * halfT * 4 * 2 * (180 / pi);
	angle->yaw  =  atan2f(2*q1*q2 + 2*q0*q3,q0*q0 + q1*q1 - q2*q2 - q3*q3) * 180 / pi;
//	angle->yaw  =  atan2f(2*q1*q2 + 2*q0*q3,q0*q0 - q1*q1 + q2*q2 - q3*q3) * 180 / pi;
//	}
}
