# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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
CMAKE_SOURCE_DIR = /home/ubuntu/legged_bot

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/legged_bot/build

# Include any dependencies generated for this target.
include CMakeFiles/legged_bot.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/legged_bot.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/legged_bot.dir/flags.make

CMakeFiles/legged_bot.dir/main.cpp.o: CMakeFiles/legged_bot.dir/flags.make
CMakeFiles/legged_bot.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/legged_bot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/legged_bot.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/legged_bot.dir/main.cpp.o -c /home/ubuntu/legged_bot/main.cpp

CMakeFiles/legged_bot.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/legged_bot.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/legged_bot/main.cpp > CMakeFiles/legged_bot.dir/main.cpp.i

CMakeFiles/legged_bot.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/legged_bot.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/legged_bot/main.cpp -o CMakeFiles/legged_bot.dir/main.cpp.s

CMakeFiles/legged_bot.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/legged_bot.dir/main.cpp.o.requires

CMakeFiles/legged_bot.dir/main.cpp.o.provides: CMakeFiles/legged_bot.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/legged_bot.dir/build.make CMakeFiles/legged_bot.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/legged_bot.dir/main.cpp.o.provides

CMakeFiles/legged_bot.dir/main.cpp.o.provides.build: CMakeFiles/legged_bot.dir/main.cpp.o


# Object files for target legged_bot
legged_bot_OBJECTS = \
"CMakeFiles/legged_bot.dir/main.cpp.o"

# External object files for target legged_bot
legged_bot_EXTERNAL_OBJECTS =

legged_bot: CMakeFiles/legged_bot.dir/main.cpp.o
legged_bot: CMakeFiles/legged_bot.dir/build.make
legged_bot: leggedlib/libleggedlib.a
legged_bot: CMakeFiles/legged_bot.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ubuntu/legged_bot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable legged_bot"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/legged_bot.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/legged_bot.dir/build: legged_bot

.PHONY : CMakeFiles/legged_bot.dir/build

CMakeFiles/legged_bot.dir/requires: CMakeFiles/legged_bot.dir/main.cpp.o.requires

.PHONY : CMakeFiles/legged_bot.dir/requires

CMakeFiles/legged_bot.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/legged_bot.dir/cmake_clean.cmake
.PHONY : CMakeFiles/legged_bot.dir/clean

CMakeFiles/legged_bot.dir/depend:
	cd /home/ubuntu/legged_bot/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/legged_bot /home/ubuntu/legged_bot /home/ubuntu/legged_bot/build /home/ubuntu/legged_bot/build /home/ubuntu/legged_bot/build/CMakeFiles/legged_bot.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/legged_bot.dir/depend

