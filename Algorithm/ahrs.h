//=====================================================================================================
//=====================================================================================================
#ifndef _AHRS_H
#define _AHRS_H

#include "stm32f4xx.h"
/*=====================================================================================================*/
/*=====================================================================================================*/
typedef struct {
	float Pitch;
	float Roll;
	float Yaw;
} EulerAngle;

typedef struct{
	int16_t		x;
	int16_t		y;
	int16_t		z;
}Accel_t;

typedef struct{
	int16_t		x;
	int16_t		y;
	int16_t		z;
}Gyro_t;

typedef struct{
	int16_t		x;
	int16_t		y;
	int16_t		z;
}Magnet_t;

extern EulerAngle AngE;
extern Accel_t Accel;
extern Gyro_t Gyro;
extern Magnet_t Magnet;
//extern float exInt, eyInt, ezInt, q0, q1, q2, q3; 
/*=====================================================================================================*/
/*=====================================================================================================*/
void AHRS_Init(void);
void AHRS_Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void AHRS_Update_test(Accel_t Accel, Gyro_t Gyro, Magnet_t Magnet); 
void toEuler(void);
/*=====================================================================================================*/
/*=====================================================================================================*/
#endif
