/*
 * pid.c
 *
 *  Created on: 2026. 3. 15.
 *      Author: juwon
 */

#include "main.h"
#include "pid.h"

// 내부 전용 함수 선언
//static void PID_CalculateErrors(PID_t *pid, float current);

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float dt,
		float target, float outMax, float outMin)
{
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->dt = dt;
	pid->target = target;
	pid->outMax = outMax;
	pid->outMin = outMin;

	if (Ki > 0)
	{
		pid->iMax = pid->outMax / Ki;
		pid->iMin = pid->outMin / Ki;
	}
	else
	{
		pid->iMax = 0;
		pid->iMin = 0;
	}

    pid->pTerm = 0.0f;
    pid->iTerm = 0.0f;
    pid->dTerm = 0.0f;
    pid->pidOutput = 0.0f;
    pid->error = 0.0f;
    pid->errorSum = 0.0f;
    pid->prevError = 0.0f;
}

void PID_SetTarget(PID_t *pid, float target)
{
	pid->target = target;
}

void PID_SetGain(PID_t *pid, float Kp, float Ki, float Kd)
{
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;

	if (Ki > 0)
	{
		pid->iMax = pid->outMax / Ki;
		pid->iMin = pid->outMin / Ki;
	}
	else
	{
		pid->iMax = 0;
		pid->iMin = 0;
	}
}

void PID_Reset(PID_t *pid)
{
	pid->pTerm = 0.0f;
	pid->iTerm = 0.0f;
	pid->dTerm = 0.0f;
	pid->pidOutput = 0.0f;
	pid->error = 0.0f;
	pid->errorSum = 0.0f;
	pid->prevError = 0.0f;
}

void PID_Compute(PID_t *pid, float current)
{
	// 1. 오차 계산
	pid->prevError = pid->error;
	pid->error = pid->target - current;

	// 2. P, D항 계산
	pid->pTerm = pid->Kp * pid->error;
	pid->dTerm = pid->Kd * ((pid->error - pid->prevError) / pid->dt);

	// 3. I항 임시 계산 (이전 주기까지 누적된 errorSum 사용)
	pid->iTerm = pid->Ki * pid->errorSum;

	// 4. 전체 제어 출력 임시 합산
	pid->pidOutput = pid->pTerm + pid->iTerm + pid->dTerm;

	// 5. Anti-windup (Conditional Integration: 조건부 적분)
	if ((pid->pidOutput >= pid->outMax && pid->error > 0) ||
	    (pid->pidOutput <= pid->outMin && pid->error < 0))
	{

	}
	else
	{
		pid->errorSum += pid->error * pid->dt;
	}

	if (pid->pidOutput > pid->outMax) pid->pidOutput = pid->outMax;
	if (pid->pidOutput < pid->outMin) pid->pidOutput = pid->outMin;
}
