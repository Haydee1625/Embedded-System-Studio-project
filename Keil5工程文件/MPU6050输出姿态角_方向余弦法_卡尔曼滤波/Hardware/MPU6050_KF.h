#ifndef __MPU6050_KF_H__
#define __MPU6050_KF_H__

void Kalman_Filter_Pitch(float pitch_acc,float* pitch_kf,float GY_kf);
void Kalman_Filter_Roll(float roll_acc,float* roll_kf,float GX_kf);

#endif
