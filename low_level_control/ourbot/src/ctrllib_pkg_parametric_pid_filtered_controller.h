#ifndef CONTROL_LIB_PARAMETRIC_PID_FILTERED_CONTROLLER_H
#define CONTROL_LIB_PARAMETRIC_PID_FILTERED_CONTROLLER_H

#include "ctrllib_pkg_parametric_pid_filtered_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

void configure_pid_filtered(config_parametric_pid_filtered_t *config, state_parametric_pid_filtered_t *state,  float Ts, float K, float Ti, float Td, float a);
void configure_pid_filtered_pkg(config_parametric_t *config, state_parametric_t *state,  float Ts, float K, float Ti, float Td, float a);
float* update_pid_filtered(config_parametric_pid_filtered_t config, state_parametric_pid_filtered_t *state, float *input);
float* update_pid_filtered_pkg(config_parametric_t config, state_parametric_t *state, float *input);

#ifdef __cplusplus
}
#endif

#endif //CONTROL_LIB_PARAMETRIC_PID_FILTERED_CONTROLLER_H
