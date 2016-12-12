#ifndef CONTROL_LIB_PARAMETRIC_PID_CONTROLLER_H
#define CONTROL_LIB_PARAMETRIC_PID_CONTROLLER_H

#include "ctrllib_pkg_parametric_pid_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

void configure_pid(config_parametric_pid_t *config, state_parametric_pid_t *state, float Ts, float P, float I, float D);
void configure_pid_pkg(config_parametric_t *config, state_parametric_t *state, float Ts, float P, float I, float D);
float* update_pid(config_parametric_pid_t config, state_parametric_pid_t *state, float *input);
float* update_pid_pkg(config_parametric_t config, state_parametric_t *state, float *input);

#ifdef __cplusplus
}
#endif

#endif //CONTROL_LIB_PARAMETRIC_PID_CONTROLLER_H
