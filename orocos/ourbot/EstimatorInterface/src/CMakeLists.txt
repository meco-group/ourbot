
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libEstimatorInterface-<target>.so
# and installs in the directory lib/orocos/EstimatorInterface/
#
orocos_component(EstimatorInterface EstimatorInterface/EstimatorInterface-component.hpp EstimatorInterface/EstimatorInterface-component.cpp SimpleEstimator/SimpleEstimator-component.hpp SimpleEstimator/SimpleEstimator-component.cpp KalmanSM/KalmanSM-component.hpp KalmanSM/KalmanSM-component.cpp)
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/EstimatorInterface/ directory
#
orocos_install_headers(EstimatorInterface/EstimatorInterface-component.hpp SimpleEstimator/SimpleEstimator-component.hpp KalmanSM/KalmanSM-component.hpp)
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libEstimatorInterface-plugin-<target>.so
# and installs in the directory lib/orocos/EstimatorInterface/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
orocos_plugin(EstimatorInterface-plugin EstimatorInterface/EstimatorInterface-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libEstimatorInterface-service-<target>.so
# and installs in the directory lib/orocos/EstimatorInterface/plugins/
#
orocos_service(EstimatorInterface-service EstimatorInterface/EstimatorInterface-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a normal library (optional):
#
# Creates a library libEstimatorInterface-support-<target>.so and installs it in
# lib/
#
orocos_library(EstimatorInterface-support EstimatorInterface/support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


