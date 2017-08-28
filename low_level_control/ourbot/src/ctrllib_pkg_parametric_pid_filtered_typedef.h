#ifndef CONTROL_LIB_PARAMETRIC_PID_FILTERED_TYPEDEF_H
#define CONTROL_LIB_PARAMETRIC_PID_FILTERED_TYPEDEF_H

typedef struct{
	float Ts;
	float a[3];
	float b[2];
} config_parametric_pid_filtered_t;

typedef struct{
	float x[2];
	float y[1];
} state_parametric_pid_filtered_t;

#endif //CONTROL_LIB_PARAMETRIC_PID_FILTERED_TYPEDEF_H
