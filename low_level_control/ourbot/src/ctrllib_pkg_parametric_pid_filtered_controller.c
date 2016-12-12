#include "ctrllib_pkg_parametric.h"

void configure_pid_filtered(config_parametric_pid_filtered_t *config, state_parametric_pid_filtered_t *state, float Ts, float K, float Ti, float Td, float a){
	config->Ts = Ts;
	
	Ti = Ti/Ts;
	Td = Td/Ts;
	float b0 = 2*Ti*(1.0f + 2.0f*a*Td);
	
	config->a[0] = K*(1.0f+2.0f*Td)*(1.0f+2.0f*Ti)/b0;
	config->a[1] = K*(2.0f - 8.0f*Td*Ti)/b0;
	config->a[2] = K*(1.0f-2.0f*Td)*(1.0f-2.0f*Ti)/b0;
	config->b[0] = -8.0f*a*Td*Ti/b0;
	config->b[1] = -2.0f*Ti*(1.0f-2.0f*a*Td)/b0;
	
	state->x[0] = 0.0f;
	state->x[1] = 0.0f;
};

/*general configure pid_filtered*/
void configure_pid_filtered_pkg(config_parametric_t *config, state_parametric_t *state, float Ts, float K, float Ti, float Td, float a){
	configure_pid_filtered(&config->pid_filtered, &state->pid_filtered, Ts, K, Ti, Td, a);
};

/*specific update pid control law*/
float* update_pid_filtered(config_parametric_pid_filtered_t config, state_parametric_pid_filtered_t *state, float *input){
	state->y[0] = state->x[0] + config.a[0]*input[0];

	state->x[0] = -config.b[0]*state->x[0] + state->x[1] + (config.a[1]-config.a[0]*config.b[0])*input[0];
	state->x[1] = -config.b[1]*state->x[0] + (config.a[2]-config.a[0]*config.b[1])*input[0];
	
	return state->y;
};

/*general update pid control law*/
float* update_pid_filtered_pkg(config_parametric_t config, state_parametric_t *state, float *input){
	return update_pid_filtered(config.pid_filtered, &state->pid_filtered, input);
};
