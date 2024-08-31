#ifndef __MPU6050_KF_H__
#define __MPU6050_KF_H__

typedef struct {
	float roll_kf;
	float pitch_kf;
	float yaw_kf;
}Angle_KF;

void Kalman_Filter_Pitch(Angle* angle,Angle_KF* angle_kf,float GY_kf);

#endif
