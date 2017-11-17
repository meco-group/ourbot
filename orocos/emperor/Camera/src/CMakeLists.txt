
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libCamera-<target>.so
# and installs in the directory lib/orocos/Camera/
#
orocos_component(Camera Camera-component.hpp Camera-component.cpp) # ...you may add multiple source files
target_link_libraries(Camera ${eagle_LIBRARIES}) # Link libraries to Orocos component here

#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/Camera/ directory
#
orocos_install_headers(Camera-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libCamera-plugin-<target>.so
# and installs in the directory lib/orocos/Camera/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
orocos_plugin(Camera-plugin Camera-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libCamera-service-<target>.so
# and installs in the directory lib/orocos/Camera/plugins/
#
orocos_service(Camera-service Camera-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a normal library (optional):
#
# Creates a library libCamera-support-<target>.so and installs it in
# lib/
#
orocos_library(Camera-support support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


