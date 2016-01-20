#include "stm32f4xx.h"
/*=====================================================================================================*/
/*=====================================================================================================*/
#ifndef _Kalman_h
#define _Kalman_h
/*=====================================================================================================*/
/*=====================================================================================================*/
#ifdef __cplusplus
 extern "C" {
#endif 
/*=====================================================================================================*/
/*=====================================================================================================*/

typedef struct {
	float	angle;// The angle calculated by the Kalman filter - part of the 2x1 state matrix
	float 	bias;// The gyro bias calculated by the Kalman filter - part of the 2x1 state matrix
	float	rate;// Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate
	float	P_00;// Error covariance matrix - This is a 2x2 matrix
	float 	P_01;
	float 	P_10;
	float 	P_11;
	float	K_0;// Kalman gain - This is a 2x1 matrix   | 1|
	float 	K_1;
	float	y;// Angle difference - 1x1 matrix           | 2|
	float	S;// Estimate error - 1x1 matrix
}kalman_t;

typedef struct {
	float	R;
	float	Q;
	float	P;
	float	x_hat;
	float	P_;
	float	K;
}kalman_single_t;

extern kalman_t kalmanX;
extern kalman_t kalmanY;
extern kalman_t kalmanZ;

extern kalman_single_t	kalman_single_X;
extern kalman_single_t	kalman_single_Y;
extern kalman_single_t	kalman_single_Z;
/*=====================================================================================================*/
/*=====================================================================================================*/
float kalman_filter_angle(kalman_t *kalman, float newAngle, float newRate, float looptime);
float kalman_single(kalman_single_t *kalman_single, float acc, float measure_noise, float process_noise);
/*=====================================================================================================*/
/*=====================================================================================================*/
#endif //Kalman.h
