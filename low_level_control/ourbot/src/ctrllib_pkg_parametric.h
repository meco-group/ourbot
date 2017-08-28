#ifndef CONTROL_LIB_PARAMETRIC_H
#define CONTROL_LIB_PARAMETRIC_H

#include "ctrllib_pkg_parametric_pid_typedef.h"
#include "ctrllib_pkg_parametric_pid_filtered_typedef.h"
//<control_lib_controller_typedef_include>//

typedef union config_parametric_t{
	config_parametric_pid_t pid;
	config_parametric_pid_filtered_t pid_filtered;
//<config_union_code>//
} config_parametric_t;

typedef union state_parametric_t{
	state_parametric_pid_t pid;
	state_parametric_pid_filtered_t pid_filtered;
//<state_union_code>//
} state_parametric_t;

#include "ctrllib_pkg_parametric_pid_controller.h"
#include "ctrllib_pkg_parametric_pid_filtered_controller.h"
//<control_lib_controller_include>//

#endif //CONTROL_LIB_PARAMETRIC_H
