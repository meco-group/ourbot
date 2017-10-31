
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libMotionPlanning-<target>.so
# and installs in the directory lib/orocos/MotionPlanning/
#

orocos_component(
MotionPlanning MotionPlanningInterface/MotionPlanningInterface-component.cpp MotionPlanningInterface/MotionPlanningInterface-component.hpp MotionPlanning/MotionPlanning-component.cpp MotionPlanning/MotionPlanning-component.hpp MotionPlanning/Problem.hpp FormationMotionPlanning/FormationMotionPlanning-component.hpp FormationMotionPlanning/FormationMotionPlanning-component.cpp FlexonomyMotionPlanning/FlexonomyMotionPlanning-component.hpp FlexonomyMotionPlanning/FlexonomyMotionPlanning-component.cpp)


#
# You may add multiple orocos_component statements.

target_link_libraries(MotionPlanning -lomg_p2p -lomg_p2pflex -lomg_p2pf -lcasadi) # Link libraries to Orocos component here

#
# Additional headers:
#
# Installs in the include/orocos/MotionPlanning/ directory
#

orocos_install_headers(MotionPlanningInterface/MotionPlanningInterface-component.hpp MotionPlanning/MotionPlanning-component.hpp MotionPlanning/Problem.hpp FormationMotionPlanning/FormationMotionPlanning-component.hpp FlexonomyMotionPlanning/FlexonomyMotionPlanning-component.hpp)

#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libMotionPlanning-plugin-<target>.so
# and installs in the directory lib/orocos/MotionPlanning/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
orocos_plugin(MotionPlanning-plugin MotionPlanningInterface/MotionPlanningInterface-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libMotionPlanning-service-<target>.so
# and installs in the directory lib/orocos/MotionPlanning/plugins/
#
orocos_service(MotionPlanning-service MotionPlanningInterface/MotionPlanningInterface-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a normal library (optional):
#
# Creates a library libMotionPlanning-support-<target>.so and installs it in
# lib/
#
orocos_library(MotionPlanning-support MotionPlanningInterface/support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


