# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.29

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = D:\softwaredownload\cmake-3.29.8-windows-x86_64\cmake-3.29.8-windows-x86_64\bin\cmake.exe

# The command to remove a file.
RM = D:\softwaredownload\cmake-3.29.8-windows-x86_64\cmake-3.29.8-windows-x86_64\bin\cmake.exe -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = D:\Omni\Omni\Chassis

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = D:\Omni\Omni\build

# Include any dependencies generated for this target.
include HW-Components/algorithms/controller/CMakeFiles/hwcomponents_algorithms_controller.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include HW-Components/algorithms/controller/CMakeFiles/hwcomponents_algorithms_controller.dir/compiler_depend.make

# Include the progress variables for this target.
include HW-Components/algorithms/controller/CMakeFiles/hwcomponents_algorithms_controller.dir/progress.make

# Include the compile flags for this target's objects.
include HW-Components/algorithms/controller/CMakeFiles/hwcomponents_algorithms_controller.dir/flags.make

HW-Components/algorithms/controller/CMakeFiles/hwcomponents_algorithms_controller.dir/pid/src/basic_pid.cpp.obj: HW-Components/algorithms/controller/CMakeFiles/hwcomponents_algorithms_controller.dir/flags.make
HW-Components/algorithms/controller/CMakeFiles/hwcomponents_algorithms_controller.dir/pid/src/basic_pid.cpp.obj: D:/Omni/Omni/Chassis/HW-Components/algorithms/controller/pid/src/basic_pid.cpp
HW-Components/algorithms/controller/CMakeFiles/hwcomponents_algorithms_controller.dir/pid/src/basic_pid.cpp.obj: HW-Components/algorithms/controller/CMakeFiles/hwcomponents_algorithms_controller.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=D:\Omni\Omni\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object HW-Components/algorithms/controller/CMakeFiles/hwcomponents_algorithms_controller.dir/pid/src/basic_pid.cpp.obj"
	cd /d D:\Omni\Omni\build\HW-Components\algorithms\controller && D:\ST\STM32CubeCLT_1.16.0\GNU-tools-for-STM32\bin\arm-none-eabi-g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT HW-Components/algorithms/controller/CMakeFiles/hwcomponents_algorithms_controller.dir/pid/src/basic_pid.cpp.obj -MF CMakeFiles\hwcomponents_algorithms_controller.dir\pid\src\basic_pid.cpp.obj.d -o CMakeFiles\hwcomponents_algorithms_controller.dir\pid\src\basic_pid.cpp.obj -c D:\Omni\Omni\Chassis\HW-Components\algorithms\controller\pid\src\basic_pid.cpp

HW-Components/algorithms/controller/CMakeFiles/hwcomponents_algorithms_controller.dir/pid/src/basic_pid.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/hwcomponents_algorithms_controller.dir/pid/src/basic_pid.cpp.i"
	cd /d D:\Omni\Omni\build\HW-Components\algorithms\controller && D:\ST\STM32CubeCLT_1.16.0\GNU-tools-for-STM32\bin\arm-none-eabi-g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E D:\Omni\Omni\Chassis\HW-Components\algorithms\controller\pid\src\basic_pid.cpp > CMakeFiles\hwcomponents_algorithms_controller.dir\pid\src\basic_pid.cpp.i

HW-Components/algorithms/controller/CMakeFiles/hwcomponents_algorithms_controller.dir/pid/src/basic_pid.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/hwcomponents_algorithms_controller.dir/pid/src/basic_pid.cpp.s"
	cd /d D:\Omni\Omni\build\HW-Components\algorithms\controller && D:\ST\STM32CubeCLT_1.16.0\GNU-tools-for-STM32\bin\arm-none-eabi-g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S D:\Omni\Omni\Chassis\HW-Components\algorithms\controller\pid\src\basic_pid.cpp -o CMakeFiles\hwcomponents_algorithms_controller.dir\pid\src\basic_pid.cpp.s

HW-Components/algorithms/controller/CMakeFiles/hwcomponents_algorithms_controller.dir/pid/src/multi_nodes_pid.cpp.obj: HW-Components/algorithms/controller/CMakeFiles/hwcomponents_algorithms_controller.dir/flags.make
HW-Components/algorithms/controller/CMakeFiles/hwcomponents_algorithms_controller.dir/pid/src/multi_nodes_pid.cpp.obj: D:/Omni/Omni/Chassis/HW-Components/algorithms/controller/pid/src/multi_nodes_pid.cpp
HW-Components/algorithms/controller/CMakeFiles/hwcomponents_algorithms_controller.dir/pid/src/multi_nodes_pid.cpp.obj: HW-Components/algorithms/controller/CMakeFiles/hwcomponents_algorithms_controller.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=D:\Omni\Omni\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object HW-Components/algorithms/controller/CMakeFiles/hwcomponents_algorithms_controller.dir/pid/src/multi_nodes_pid.cpp.obj"
	cd /d D:\Omni\Omni\build\HW-Components\algorithms\controller && D:\ST\STM32CubeCLT_1.16.0\GNU-tools-for-STM32\bin\arm-none-eabi-g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT HW-Components/algorithms/controller/CMakeFiles/hwcomponents_algorithms_controller.dir/pid/src/multi_nodes_pid.cpp.obj -MF CMakeFiles\hwcomponents_algorithms_controller.dir\pid\src\multi_nodes_pid.cpp.obj.d -o CMakeFiles\hwcomponents_algorithms_controller.dir\pid\src\multi_nodes_pid.cpp.obj -c D:\Omni\Omni\Chassis\HW-Components\algorithms\controller\pid\src\multi_nodes_pid.cpp

HW-Components/algorithms/controller/CMakeFiles/hwcomponents_algorithms_controller.dir/pid/src/multi_nodes_pid.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/hwcomponents_algorithms_controller.dir/pid/src/multi_nodes_pid.cpp.i"
	cd /d D:\Omni\Omni\build\HW-Components\algorithms\controller && D:\ST\STM32CubeCLT_1.16.0\GNU-tools-for-STM32\bin\arm-none-eabi-g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E D:\Omni\Omni\Chassis\HW-Components\algorithms\controller\pid\src\multi_nodes_pid.cpp > CMakeFiles\hwcomponents_algorithms_controller.dir\pid\src\multi_nodes_pid.cpp.i

HW-Components/algorithms/controller/CMakeFiles/hwcomponents_algorithms_controller.dir/pid/src/multi_nodes_pid.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/hwcomponents_algorithms_controller.dir/pid/src/multi_nodes_pid.cpp.s"
	cd /d D:\Omni\Omni\build\HW-Components\algorithms\controller && D:\ST\STM32CubeCLT_1.16.0\GNU-tools-for-STM32\bin\arm-none-eabi-g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S D:\Omni\Omni\Chassis\HW-Components\algorithms\controller\pid\src\multi_nodes_pid.cpp -o CMakeFiles\hwcomponents_algorithms_controller.dir\pid\src\multi_nodes_pid.cpp.s

# Object files for target hwcomponents_algorithms_controller
hwcomponents_algorithms_controller_OBJECTS = \
"CMakeFiles/hwcomponents_algorithms_controller.dir/pid/src/basic_pid.cpp.obj" \
"CMakeFiles/hwcomponents_algorithms_controller.dir/pid/src/multi_nodes_pid.cpp.obj"

# External object files for target hwcomponents_algorithms_controller
hwcomponents_algorithms_controller_EXTERNAL_OBJECTS =

D:/Omni/Omni/Chassis/Lib/hwcomponents/algorithms/controller/libhwcomponents_algorithms_controller.a: HW-Components/algorithms/controller/CMakeFiles/hwcomponents_algorithms_controller.dir/pid/src/basic_pid.cpp.obj
D:/Omni/Omni/Chassis/Lib/hwcomponents/algorithms/controller/libhwcomponents_algorithms_controller.a: HW-Components/algorithms/controller/CMakeFiles/hwcomponents_algorithms_controller.dir/pid/src/multi_nodes_pid.cpp.obj
D:/Omni/Omni/Chassis/Lib/hwcomponents/algorithms/controller/libhwcomponents_algorithms_controller.a: HW-Components/algorithms/controller/CMakeFiles/hwcomponents_algorithms_controller.dir/build.make
D:/Omni/Omni/Chassis/Lib/hwcomponents/algorithms/controller/libhwcomponents_algorithms_controller.a: HW-Components/algorithms/controller/CMakeFiles/hwcomponents_algorithms_controller.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=D:\Omni\Omni\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library D:\Omni\Omni\Chassis\Lib\hwcomponents\algorithms\controller\libhwcomponents_algorithms_controller.a"
	cd /d D:\Omni\Omni\build\HW-Components\algorithms\controller && $(CMAKE_COMMAND) -P CMakeFiles\hwcomponents_algorithms_controller.dir\cmake_clean_target.cmake
	cd /d D:\Omni\Omni\build\HW-Components\algorithms\controller && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\hwcomponents_algorithms_controller.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
HW-Components/algorithms/controller/CMakeFiles/hwcomponents_algorithms_controller.dir/build: D:/Omni/Omni/Chassis/Lib/hwcomponents/algorithms/controller/libhwcomponents_algorithms_controller.a
.PHONY : HW-Components/algorithms/controller/CMakeFiles/hwcomponents_algorithms_controller.dir/build

HW-Components/algorithms/controller/CMakeFiles/hwcomponents_algorithms_controller.dir/clean:
	cd /d D:\Omni\Omni\build\HW-Components\algorithms\controller && $(CMAKE_COMMAND) -P CMakeFiles\hwcomponents_algorithms_controller.dir\cmake_clean.cmake
.PHONY : HW-Components/algorithms/controller/CMakeFiles/hwcomponents_algorithms_controller.dir/clean

HW-Components/algorithms/controller/CMakeFiles/hwcomponents_algorithms_controller.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" D:\Omni\Omni\Chassis D:\Omni\Omni\Chassis\HW-Components\algorithms\controller D:\Omni\Omni\build D:\Omni\Omni\build\HW-Components\algorithms\controller D:\Omni\Omni\build\HW-Components\algorithms\controller\CMakeFiles\hwcomponents_algorithms_controller.dir\DependInfo.cmake "--color=$(COLOR)"
.PHONY : HW-Components/algorithms/controller/CMakeFiles/hwcomponents_algorithms_controller.dir/depend
