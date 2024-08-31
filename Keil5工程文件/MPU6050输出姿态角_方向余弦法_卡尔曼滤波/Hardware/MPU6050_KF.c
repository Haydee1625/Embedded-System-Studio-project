#include "stm32f10x.h"                  // Device header
#include "MPU6050_PC.h"
#include "MPU6050_KF.h"

static float Q_angle = 0.001f;		//角度的过程误差协方差
static float Q_gyro  = 0.003f;		//漂移角速度的过程误差协方差  
static float R_angle = 0.5f;		//角度的测量误差协方差
static float dt      = 0.01f;		//采样周期即计算任务周期10ms

void Kalman_Filter_Pitch(float pitch_acc,float* pitch_kf,float GY_kf)
{
	
	static float w_bias_y = 0.0f;							//陀螺仪角速度的偏差
	static float K_Pitch_0, K_Pitch_1;				//卡尔曼增益
	static float P_P[2][2] = { { 1, 0 },{ 0, 1 } }; //过程协方差矩阵P，初始值为单位阵
	/*
	1. 先验估计
* * *公式1：X(k|k-1) = AX(k-1|k-1) + BU(k) + (W(k))

		X = (Angle,Q_bias)
		A(1,1) = 1,A(1,2) = -dt
		A(2,1) = 0,A(2,2) = 1
		注：上下连“[”代表矩阵
		预测当前角度值：
		[ angle ] 	[1 -dt][ angle ]   [dt]
		[ w_bias] = [0  1 ][ w_bias] + [ 0] * newGyro
		故
		angle = angle - w_bias*dt + newGyro * dt
		w_bias = w_bias
	*/
	float pitch_est = *pitch_kf + (GY_kf - w_bias_y) * dt; //状态方程,角度值等于上次最优角度加角速度减零漂后积分
	w_bias_y = w_bias_y;
	
	/*
	2. 先验误差协方差矩阵
* * *公式2：P(k|k-1)=AP(k-1|k-1)A^T + Q 
	*/
	//由于dt^2太小，故dt^2省略
	P_P[0][0] = P_P[0][0] + Q_angle - (P_P[0][1] + P_P[1][0])*dt;
	P_P[0][1] = P_P[0][1] - P_P[1][1]*dt;
	P_P[1][0] = P_P[1][0] - P_P[1][1]*dt;
	P_P[1][1] = P_P[1][1] + Q_gyro; 
	
	/*
	3. 计算卡尔曼增益
* * *公式3：Kg(k)= P(k|k-1)H^T/(HP(k|k-1)H^T+R)
				Kg = (K_0,K_1) 对应angle,Q_bias增益
				H = (1,0)
	*/
	K_Pitch_0 = P_P[0][0] / (P_P[0][0] + R_angle);
	K_Pitch_1 = P_P[1][0] / (P_P[0][0] + R_angle);
	
	/*
	4. 计算当前最优化估计值
* * *公式4：X(k|k) = X(k|k-1) + kg(k)[z(k) - HX(k|k-1)]
		angle = angle + K_0*(newAngle - angle)
		Q_bias = Q_bias + K_1*(newAngle - angle)
	*/
		
	*pitch_kf = pitch_est + K_Pitch_0 * (pitch_acc - pitch_est);
	w_bias_y = w_bias_y + K_Pitch_1 * (pitch_acc - pitch_est);
	
	/*
	5. 更新协方差矩阵
* * *公式5：P(k|k)=[I-Kg(k)H]P(k|k-1)
	*/
	P_P[1][0] = P_P[1][0] - K_Pitch_1 * P_P[0][0];
	P_P[0][0] = P_P[0][0] - K_Pitch_0 * P_P[0][0];
	P_P[1][1] = P_P[1][1] - K_Pitch_1 * P_P[0][1];
	P_P[0][1] = P_P[0][1] - K_Pitch_0 * P_P[0][1];
}

void Kalman_Filter_Roll(float roll_acc,float* roll_kf,float GX_kf)
{
	static float w_bias_x = 0.0f;				           //陀螺仪角速度的偏差
	static float K_Roll_0, K_Roll_1;				           //卡尔曼增益
	static float P_R[2][2] = { { 1, 0 },{ 0, 1 } };//过程协方差矩阵P，初始值为单位阵


	float roll_est = *roll_kf + (GX_kf - w_bias_x) * dt; //状态方程,角度值等于上次最优角度加角速度减零漂后积分
	w_bias_x = w_bias_x;

	P_R[0][0] = P_R[0][0] + Q_angle - (P_R[0][1] + P_R[1][0])*dt;
	P_R[0][1] = P_R[0][1] - P_R[1][1]*dt;
	P_R[1][0] = P_R[1][0] - P_R[1][1]*dt;
	P_R[1][1] = P_R[1][1] + Q_gyro; 
	
	K_Roll_0 = P_R[0][0] / (P_R[0][0] + R_angle);
	K_Roll_1 = P_R[1][0] / (P_R[0][0] + R_angle);
	
		
	*roll_kf = roll_est + K_Roll_0 * (roll_acc - roll_est);
	w_bias_x = w_bias_x + K_Roll_1 * (roll_acc - roll_est);
	
	P_R[1][0] = P_R[1][0] - K_Roll_1 * P_R[0][0];
	P_R[0][0] = P_R[0][0] - K_Roll_0 * P_R[0][0];
	P_R[1][1] = P_R[1][1] - K_Roll_1 * P_R[0][1];
	P_R[0][1] = P_R[0][1] - K_Roll_0 * P_R[0][1];
}
