#include "PID.h"

void updatePID(PID_Controller* pid, double current) {
	pid->CurrentError=pid->target - current;
	double P,D;
	P = pid->Kp * pid->CurrentError;
	pid->integral += pid->Ki * pid->CurrentError * (pid->dt/1000);
	D = pid->Kd * (pid->CurrentError - pid->prev_error) / (pid->dt/1000);
	pid->output = P + pid->integral + D;
	// Integral with anti-windup
	if (pid->integral > pid->max_Integral) {
		pid->integral = pid->max_Integral;
	} else if (pid->integral < pid->min_Integral) {
		pid->integral = pid->min_Integral;
	}
	// Saturate output within limits
	if (pid->output > pid->max_output) {
		pid->output = pid->max_output;
	} else if (pid->output < pid->min_output) {
		pid->output = pid->min_output;
	}
	pid->prev_error = pid->CurrentError;
}
