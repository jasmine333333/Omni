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
include HW-Components/devices/servo/CMakeFiles/hwcomponents_devices_servo.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include HW-Components/devices/servo/CMakeFiles/hwcomponents_devices_servo.dir/compiler_depend.make

# Include the progress variables for this target.
include HW-Components/devices/servo/CMakeFiles/hwcomponents_devices_servo.dir/progress.make

# Include the compile flags for this target's objects.
include HW-Components/devices/servo/CMakeFiles/hwcomponents_devices_servo.dir/flags.make

HW-Components/devices/servo/CMakeFiles/hwcomponents_devices_servo.dir/src/servo.cpp.obj: HW-Components/devices/servo/CMakeFiles/hwcomponents_devices_servo.dir/flags.make
HW-Components/devices/servo/CMakeFiles/hwcomponents_devices_servo.dir/src/servo.cpp.obj: D:/Omni/Omni/Chassis/HW-Components/devices/servo/src/servo.cpp
HW-Components/devices/servo/CMakeFiles/hwcomponents_devices_servo.dir/src/servo.cpp.obj: HW-Components/devices/servo/CMakeFiles/hwcomponents_devices_servo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=D:\Omni\Omni\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object HW-Components/devices/servo/CMakeFiles/hwcomponents_devices_servo.dir/src/servo.cpp.obj"
	cd /d D:\Omni\Omni\build\HW-Components\devices\servo && D:\ST\STM32CubeCLT_1.16.0\GNU-tools-for-STM32\bin\arm-none-eabi-g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT HW-Components/devices/servo/CMakeFiles/hwcomponents_devices_servo.dir/src/servo.cpp.obj -MF CMakeFiles\hwcomponents_devices_servo.dir\src\servo.cpp.obj.d -o CMakeFiles\hwcomponents_devices_servo.dir\src\servo.cpp.obj -c D:\Omni\Omni\Chassis\HW-Components\devices\servo\src\servo.cpp

HW-Components/devices/servo/CMakeFiles/hwcomponents_devices_servo.dir/src/servo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/hwcomponents_devices_servo.dir/src/servo.cpp.i"
	cd /d D:\Omni\Omni\build\HW-Components\devices\servo && D:\ST\STM32CubeCLT_1.16.0\GNU-tools-for-STM32\bin\arm-none-eabi-g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E D:\Omni\Omni\Chassis\HW-Components\devices\servo\src\servo.cpp > CMakeFiles\hwcomponents_devices_servo.dir\src\servo.cpp.i

HW-Components/devices/servo/CMakeFiles/hwcomponents_devices_servo.dir/src/servo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/hwcomponents_devices_servo.dir/src/servo.cpp.s"
	cd /d D:\Omni\Omni\build\HW-Components\devices\servo && D:\ST\STM32CubeCLT_1.16.0\GNU-tools-for-STM32\bin\arm-none-eabi-g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S D:\Omni\Omni\Chassis\HW-Components\devices\servo\src\servo.cpp -o CMakeFiles\hwcomponents_devices_servo.dir\src\servo.cpp.s

# Object files for target hwcomponents_devices_servo
hwcomponents_devices_servo_OBJECTS = \
"CMakeFiles/hwcomponents_devices_servo.dir/src/servo.cpp.obj"

# External object files for target hwcomponents_devices_servo
hwcomponents_devices_servo_EXTERNAL_OBJECTS =

D:/Omni/Omni/Chassis/Lib/hwcomponents/devices/servo/libhwcomponents_devices_servo.a: HW-Components/devices/servo/CMakeFiles/hwcomponents_devices_servo.dir/src/servo.cpp.obj
D:/Omni/Omni/Chassis/Lib/hwcomponents/devices/servo/libhwcomponents_devices_servo.a: HW-Components/devices/servo/CMakeFiles/hwcomponents_devices_servo.dir/build.make
D:/Omni/Omni/Chassis/Lib/hwcomponents/devices/servo/libhwcomponents_devices_servo.a: HW-Components/devices/servo/CMakeFiles/hwcomponents_devices_servo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=D:\Omni\Omni\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library D:\Omni\Omni\Chassis\Lib\hwcomponents\devices\servo\libhwcomponents_devices_servo.a"
	cd /d D:\Omni\Omni\build\HW-Components\devices\servo && $(CMAKE_COMMAND) -P CMakeFiles\hwcomponents_devices_servo.dir\cmake_clean_target.cmake
	cd /d D:\Omni\Omni\build\HW-Components\devices\servo && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\hwcomponents_devices_servo.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
HW-Components/devices/servo/CMakeFiles/hwcomponents_devices_servo.dir/build: D:/Omni/Omni/Chassis/Lib/hwcomponents/devices/servo/libhwcomponents_devices_servo.a
.PHONY : HW-Components/devices/servo/CMakeFiles/hwcomponents_devices_servo.dir/build

HW-Components/devices/servo/CMakeFiles/hwcomponents_devices_servo.dir/clean:
	cd /d D:\Omni\Omni\build\HW-Components\devices\servo && $(CMAKE_COMMAND) -P CMakeFiles\hwcomponents_devices_servo.dir\cmake_clean.cmake
.PHONY : HW-Components/devices/servo/CMakeFiles/hwcomponents_devices_servo.dir/clean

HW-Components/devices/servo/CMakeFiles/hwcomponents_devices_servo.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" D:\Omni\Omni\Chassis D:\Omni\Omni\Chassis\HW-Components\devices\servo D:\Omni\Omni\build D:\Omni\Omni\build\HW-Components\devices\servo D:\Omni\Omni\build\HW-Components\devices\servo\CMakeFiles\hwcomponents_devices_servo.dir\DependInfo.cmake "--color=$(COLOR)"
.PHONY : HW-Components/devices/servo/CMakeFiles/hwcomponents_devices_servo.dir/depend

