
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libHawkEye-<target>.so
# and installs in the directory lib/orocos/HawkEye/
#
orocos_component(HawkEye HawkEye-component.hpp HawkEye-component.cpp Obstacle.hpp Obstacle.cpp Circle.hpp Circle.cpp Rectangle.hpp Rectangle.cpp) # ...you may add multiple source files
target_link_libraries(HawkEye ${OpenCV_LIBRARIES}) # Link libraries to Orocos component here

#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/HawkEye/ directory
#
orocos_install_headers(HawkEye-component.hpp Obstacle.hpp Circle.hpp Rectangle.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libHawkEye-plugin-<target>.so
# and installs in the directory lib/orocos/HawkEye/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
orocos_plugin(HawkEye-plugin HawkEye-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libHawkEye-service-<target>.so
# and installs in the directory lib/orocos/HawkEye/plugins/
#
orocos_service(HawkEye-service HawkEye-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a normal library (optional):
#
# Creates a library libHawkEye-support-<target>.so and installs it in
# lib/
#
orocos_library(HawkEye-support support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


