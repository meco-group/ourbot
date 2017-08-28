#ifndef CONTROL_LIB_PARAMETRIC_PID_TYPEDEF_H
#define CONTROL_LIB_PARAMETRIC_PID_TYPEDEF_H

typedef struct{
	float Ts;
	float P;
	float I;
	float D;
} config_parametric_pid_t;

typedef struct{
	float x[2]; //Xint = x[0]; Xold = x[1]
	float y[1];
} state_parametric_pid_t;

#endif //CONTROL_LIB_PARAMETRIC_PID_TYPEDEF_H
