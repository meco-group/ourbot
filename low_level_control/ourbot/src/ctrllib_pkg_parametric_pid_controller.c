#include "ctrllib_pkg_parametric.h"

/*specific configure pid*/
void configure_pid(config_parametric_pid_t *config, state_parametric_pid_t *state, float Ts, float P, float I, float D){
	config->Ts = Ts;
	config->P = P;
	config->I = I;
	config->D = D;
	
	state->x[0] = 0.0f;
	state->x[1] = 0.0f;
};

/*general configure pid*/
void configure_pid_pkg(config_parametric_t *config, state_parametric_t *state, float Ts, float P, float I, float D){
	configure_pid(&config->pid, &state->pid, Ts, P, I, D);
};

/*specific update pid control law*/
float* update_pid(config_parametric_pid_t config, state_parametric_pid_t *state, float *input){
	state->x[0] += ( input[0] + state->x[1] ) * config.Ts / 2.0f; //update integral state
	state->y[0] = config.P * input[0] + config.I * state->x[0] + config.D * (input[0] - state->x[1])/config.Ts; //calculate output
	state->x[1] = input[0]; //update old error state
	
	return state->y;
};

/*general update pid control law*/
float* update_pid_pkg(config_parametric_t config, state_parametric_t *state, float *input){
	return update_pid(config.pid, &state->pid, input);
};
