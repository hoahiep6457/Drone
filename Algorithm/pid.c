#include "stm32f4xx.h"
#include "main.h"
#include <stdlib.h>
#include <math.h>
#include "pid.h"
/*=====================================================================================================*/
/*=====================================================================================================*/
pid_t PID_Pitch;
pid_t PID_Roll;
pid_t PID_Yaw;
/*=====================================================================================================*/
/*=====================================================================================================*/
float PID_Update(pid_t *pid, float setpoint, float target)
{
	float error = setpoint - target;
	float diff =  error - pid->lastError;
	pid->lastError =  error; //save value for next time
	//calculate PID contents
	pid->Kp_Value = pid->KP*error;
	pid->Ki_Value += pid->KI*error*pid_sampletime;
	pid->Kd_Value = pid->KD*diff*inv_pid_sampletime;
	//Update ouput
	pid->Output = pid->Kp_Value + pid->Ki_Value + pid->Kd_Value;
	if (pid->Output > Outmax){
		pid->Output = Outmax;
	}
	if (pid->Output < Outmin){
		pid->Output = Outmin;
	}
	return pid->Output;
}
/*=====================================================================================================*/
/*=====================================================================================================*/
void PID_init(pid_t *pid, float kp, float ki, float kd)
{
	pid->KP = kp;
	pid->KI = ki;
	pid->KD = kd;
}
/*=====================================================================================================*/
/*=====================================================================================================*/