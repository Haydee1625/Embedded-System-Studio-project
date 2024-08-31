#ifndef __MPU6050_PC_H__
#define __MPU6050_PC_H__

typedef struct {
	float roll;
	float pitch;
	float yaw;
}Angle;

void MPU6050_DataConvert(float * AX_Convert,float *  AY_Convert,float *  AZ_Convert,
	float * GX_Convert,float *  GY_Convert,float *  GZ_Convert,int16_t AX,int16_t AY,
		int16_t AZ,int16_t GX,int16_t GY,int16_t GZ);

void Pose_Calculating(float AX_Convert, float AY_Convert, float AZ_Convert, float GX_Convert, 
	float GY_Convert, float GZ_Convert,Angle* angle); 

void ACC_calculateAngles(float ax, float ay, float az, float *pitch, float *roll); 

#endif
