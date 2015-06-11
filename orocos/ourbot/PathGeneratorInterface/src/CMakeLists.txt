
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libPathGeneratorInterface-<target>.so
# and installs in the directory lib/orocos/PathGeneratorInterface/
#
orocos_component(PathGeneratorInterface PathGeneratorInterface/PathGeneratorInterface-component.hpp PathGeneratorInterface/PathGeneratorInterface-component.cpp ExamplePathGenerator/ExamplePathGenerator-component.hpp ExamplePathGenerator/ExamplePathGenerator-component.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/PathGeneratorInterface/ directory
#
orocos_install_headers(PathGeneratorInterface/PathGeneratorInterface-component.hpp ExamplePathGenerator/ExamplePathGenerator-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libPathGeneratorInterface-plugin-<target>.so
# and installs in the directory lib/orocos/PathGeneratorInterface/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
orocos_plugin(PathGeneratorInterface-plugin PathGeneratorInterface/PathGeneratorInterface-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libPathGeneratorInterface-service-<target>.so
# and installs in the directory lib/orocos/PathGeneratorInterface/plugins/
#
orocos_service(PathGeneratorInterface-service PathGeneratorInterface/PathGeneratorInterface-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a normal library (optional):
#
# Creates a library libPathGeneratorInterface-support-<target>.so and installs it in
# lib/
#
orocos_library(PathGeneratorInterface-support PathGeneratorInterface/support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


