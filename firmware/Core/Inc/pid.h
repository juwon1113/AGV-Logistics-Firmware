/*
 * pid.h
 *
 *  Created on: 2026. 3. 15.
 *      Author: juwon
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct
{
	float Kp, Ki, Kd;
	float dt;
	float target;
	float outMax, outMin;
	float iMax, iMin;
	float pTerm;
	float iTerm;
	float dTerm;
	float pidOutput;
	float error;
	float errorSum;
	float prevError;
} PID_t;

// PID Initial Function
void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float dt,
		float target, float outMax, float outMin);

// PID Set Target Function
void PID_SetTarget(PID_t *pid, float target);

// PID Set p, i, d Gain Function
void PID_SetGain(PID_t *pid, float Kp, float Ki, float Kd);

// PID internal state Reset Function (errorSum, prevError 등 초기화, gain/limit은 유지)
void PID_Reset(PID_t *pid);

// PID Control Compute System
void PID_Compute(PID_t *pid, float current);

#endif /* INC_PID_H_ */
