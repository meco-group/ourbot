# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/maarten/projects/mavlink_new/mavlink

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/maarten/projects/mavlink_new/mavlink/build

# Utility rule file for slugs.xml-v0.9.

# Include the progress variables for this target.
include CMakeFiles/slugs.xml-v0.9.dir/progress.make

CMakeFiles/slugs.xml-v0.9: slugs.xml-v0.9-stamp

slugs.xml-v0.9-stamp: ../message_definitions/v0.9/slugs.xml
slugs.xml-v0.9-stamp: ../pymavlink/generator/mavgen.py
	$(CMAKE_COMMAND) -E cmake_progress_report /home/maarten/projects/mavlink_new/mavlink/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating slugs.xml-v0.9-stamp"
	PYTHONPATH=:/home/maarten/projects/mavlink_new/mavlink /usr/bin/python /home/maarten/projects/mavlink_new/mavlink/pymavlink/generator/mavgen.py --lang=C --wire-protocol=0.9 --output=include/v0.9 /home/maarten/projects/mavlink_new/mavlink/message_definitions/v0.9/slugs.xml
	touch slugs.xml-v0.9-stamp

slugs.xml-v0.9: CMakeFiles/slugs.xml-v0.9
slugs.xml-v0.9: slugs.xml-v0.9-stamp
slugs.xml-v0.9: CMakeFiles/slugs.xml-v0.9.dir/build.make
.PHONY : slugs.xml-v0.9

# Rule to build all files generated by this target.
CMakeFiles/slugs.xml-v0.9.dir/build: slugs.xml-v0.9
.PHONY : CMakeFiles/slugs.xml-v0.9.dir/build

CMakeFiles/slugs.xml-v0.9.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/slugs.xml-v0.9.dir/cmake_clean.cmake
.PHONY : CMakeFiles/slugs.xml-v0.9.dir/clean

CMakeFiles/slugs.xml-v0.9.dir/depend:
	cd /home/maarten/projects/mavlink_new/mavlink/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/maarten/projects/mavlink_new/mavlink /home/maarten/projects/mavlink_new/mavlink /home/maarten/projects/mavlink_new/mavlink/build /home/maarten/projects/mavlink_new/mavlink/build /home/maarten/projects/mavlink_new/mavlink/build/CMakeFiles/slugs.xml-v0.9.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/slugs.xml-v0.9.dir/depend
