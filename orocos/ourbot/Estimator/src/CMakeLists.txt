
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libEstimator-<target>.so
# and installs in the directory lib/orocos/Estimator/
#
orocos_component(Estimator EstimatorInterface/EstimatorInterface-component.hpp EstimatorInterface/EstimatorInterface-component.cpp Kalman/Kalman-component.hpp Kalman/Kalman-component.cpp Kalman/kalman_odometry.cpp Kalman/kalman_odometry.hpp Kalman/kalman_odometry_impl.hpp Kalman/kalman_kinematic.hpp Kalman/kalman_filter.hpp Kalman/kalman_operations.cpp Kalman/kalman_operations.hpp)
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/Estimator/ directory
#
orocos_install_headers(EstimatorInterface/EstimatorInterface-component.hpp Kalman/Kalman-component.hpp Kalman/kalman_odometry.hpp Kalman/kalman_odometry_impl.hpp Kalman/kalman_kinematic.hpp Kalman/kalman_filter.hpp Kalman/kalman_operations.hpp)
#
# You may add multiple orocos_install_headers statements.

target_link_libraries (Estimator ${SLICOT_LIB})

#
# Building a Plugin
#
# Creates a plugin library libEstimator-plugin-<target>.so
# and installs in the directory lib/orocos/Estimator/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
orocos_plugin(Estimator-plugin EstimatorInterface/EstimatorInterface-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libEstimator-service-<target>.so
# and installs in the directory lib/orocos/Estimator/plugins/
#
orocos_service(Estimator-service EstimatorInterface/EstimatorInterface-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a normal library (optional):
#
# Creates a library libEstimator-support-<target>.so and installs it in
# lib/
#
orocos_library(Estimator-support EstimatorInterface/support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


