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
include HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/compiler_depend.make

# Include the progress variables for this target.
include HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/progress.make

# Include the compile flags for this target's objects.
include HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/flags.make

HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/referee.cpp.obj: HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/flags.make
HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/referee.cpp.obj: D:/Omni/Omni/Chassis/HW-Components/devices/referee/src/referee.cpp
HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/referee.cpp.obj: HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=D:\Omni\Omni\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/referee.cpp.obj"
	cd /d D:\Omni\Omni\build\HW-Components\devices\referee && D:\ST\STM32CubeCLT_1.16.0\GNU-tools-for-STM32\bin\arm-none-eabi-g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/referee.cpp.obj -MF CMakeFiles\hwcomponents_devices_referee.dir\src\referee.cpp.obj.d -o CMakeFiles\hwcomponents_devices_referee.dir\src\referee.cpp.obj -c D:\Omni\Omni\Chassis\HW-Components\devices\referee\src\referee.cpp

HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/referee.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/hwcomponents_devices_referee.dir/src/referee.cpp.i"
	cd /d D:\Omni\Omni\build\HW-Components\devices\referee && D:\ST\STM32CubeCLT_1.16.0\GNU-tools-for-STM32\bin\arm-none-eabi-g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E D:\Omni\Omni\Chassis\HW-Components\devices\referee\src\referee.cpp > CMakeFiles\hwcomponents_devices_referee.dir\src\referee.cpp.i

HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/referee.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/hwcomponents_devices_referee.dir/src/referee.cpp.s"
	cd /d D:\Omni\Omni\build\HW-Components\devices\referee && D:\ST\STM32CubeCLT_1.16.0\GNU-tools-for-STM32\bin\arm-none-eabi-g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S D:\Omni\Omni\Chassis\HW-Components\devices\referee\src\referee.cpp -o CMakeFiles\hwcomponents_devices_referee.dir\src\referee.cpp.s

HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_crc.cpp.obj: HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/flags.make
HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_crc.cpp.obj: D:/Omni/Omni/Chassis/HW-Components/devices/referee/src/rfr_crc.cpp
HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_crc.cpp.obj: HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=D:\Omni\Omni\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_crc.cpp.obj"
	cd /d D:\Omni\Omni\build\HW-Components\devices\referee && D:\ST\STM32CubeCLT_1.16.0\GNU-tools-for-STM32\bin\arm-none-eabi-g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_crc.cpp.obj -MF CMakeFiles\hwcomponents_devices_referee.dir\src\rfr_crc.cpp.obj.d -o CMakeFiles\hwcomponents_devices_referee.dir\src\rfr_crc.cpp.obj -c D:\Omni\Omni\Chassis\HW-Components\devices\referee\src\rfr_crc.cpp

HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_crc.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_crc.cpp.i"
	cd /d D:\Omni\Omni\build\HW-Components\devices\referee && D:\ST\STM32CubeCLT_1.16.0\GNU-tools-for-STM32\bin\arm-none-eabi-g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E D:\Omni\Omni\Chassis\HW-Components\devices\referee\src\rfr_crc.cpp > CMakeFiles\hwcomponents_devices_referee.dir\src\rfr_crc.cpp.i

HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_crc.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_crc.cpp.s"
	cd /d D:\Omni\Omni\build\HW-Components\devices\referee && D:\ST\STM32CubeCLT_1.16.0\GNU-tools-for-STM32\bin\arm-none-eabi-g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S D:\Omni\Omni\Chassis\HW-Components\devices\referee\src\rfr_crc.cpp -o CMakeFiles\hwcomponents_devices_referee.dir\src\rfr_crc.cpp.s

HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_decoder.cpp.obj: HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/flags.make
HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_decoder.cpp.obj: D:/Omni/Omni/Chassis/HW-Components/devices/referee/src/rfr_decoder.cpp
HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_decoder.cpp.obj: HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=D:\Omni\Omni\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_decoder.cpp.obj"
	cd /d D:\Omni\Omni\build\HW-Components\devices\referee && D:\ST\STM32CubeCLT_1.16.0\GNU-tools-for-STM32\bin\arm-none-eabi-g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_decoder.cpp.obj -MF CMakeFiles\hwcomponents_devices_referee.dir\src\rfr_decoder.cpp.obj.d -o CMakeFiles\hwcomponents_devices_referee.dir\src\rfr_decoder.cpp.obj -c D:\Omni\Omni\Chassis\HW-Components\devices\referee\src\rfr_decoder.cpp

HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_decoder.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_decoder.cpp.i"
	cd /d D:\Omni\Omni\build\HW-Components\devices\referee && D:\ST\STM32CubeCLT_1.16.0\GNU-tools-for-STM32\bin\arm-none-eabi-g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E D:\Omni\Omni\Chassis\HW-Components\devices\referee\src\rfr_decoder.cpp > CMakeFiles\hwcomponents_devices_referee.dir\src\rfr_decoder.cpp.i

HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_decoder.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_decoder.cpp.s"
	cd /d D:\Omni\Omni\build\HW-Components\devices\referee && D:\ST\STM32CubeCLT_1.16.0\GNU-tools-for-STM32\bin\arm-none-eabi-g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S D:\Omni\Omni\Chassis\HW-Components\devices\referee\src\rfr_decoder.cpp -o CMakeFiles\hwcomponents_devices_referee.dir\src\rfr_decoder.cpp.s

HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_encoder.cpp.obj: HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/flags.make
HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_encoder.cpp.obj: D:/Omni/Omni/Chassis/HW-Components/devices/referee/src/rfr_encoder.cpp
HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_encoder.cpp.obj: HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=D:\Omni\Omni\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_encoder.cpp.obj"
	cd /d D:\Omni\Omni\build\HW-Components\devices\referee && D:\ST\STM32CubeCLT_1.16.0\GNU-tools-for-STM32\bin\arm-none-eabi-g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_encoder.cpp.obj -MF CMakeFiles\hwcomponents_devices_referee.dir\src\rfr_encoder.cpp.obj.d -o CMakeFiles\hwcomponents_devices_referee.dir\src\rfr_encoder.cpp.obj -c D:\Omni\Omni\Chassis\HW-Components\devices\referee\src\rfr_encoder.cpp

HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_encoder.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_encoder.cpp.i"
	cd /d D:\Omni\Omni\build\HW-Components\devices\referee && D:\ST\STM32CubeCLT_1.16.0\GNU-tools-for-STM32\bin\arm-none-eabi-g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E D:\Omni\Omni\Chassis\HW-Components\devices\referee\src\rfr_encoder.cpp > CMakeFiles\hwcomponents_devices_referee.dir\src\rfr_encoder.cpp.i

HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_encoder.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_encoder.cpp.s"
	cd /d D:\Omni\Omni\build\HW-Components\devices\referee && D:\ST\STM32CubeCLT_1.16.0\GNU-tools-for-STM32\bin\arm-none-eabi-g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S D:\Omni\Omni\Chassis\HW-Components\devices\referee\src\rfr_encoder.cpp -o CMakeFiles\hwcomponents_devices_referee.dir\src\rfr_encoder.cpp.s

HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_pkg_0x0301_inter_among_robots.cpp.obj: HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/flags.make
HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_pkg_0x0301_inter_among_robots.cpp.obj: D:/Omni/Omni/Chassis/HW-Components/devices/referee/src/rfr_pkg_0x0301_inter_among_robots.cpp
HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_pkg_0x0301_inter_among_robots.cpp.obj: HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=D:\Omni\Omni\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_pkg_0x0301_inter_among_robots.cpp.obj"
	cd /d D:\Omni\Omni\build\HW-Components\devices\referee && D:\ST\STM32CubeCLT_1.16.0\GNU-tools-for-STM32\bin\arm-none-eabi-g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_pkg_0x0301_inter_among_robots.cpp.obj -MF CMakeFiles\hwcomponents_devices_referee.dir\src\rfr_pkg_0x0301_inter_among_robots.cpp.obj.d -o CMakeFiles\hwcomponents_devices_referee.dir\src\rfr_pkg_0x0301_inter_among_robots.cpp.obj -c D:\Omni\Omni\Chassis\HW-Components\devices\referee\src\rfr_pkg_0x0301_inter_among_robots.cpp

HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_pkg_0x0301_inter_among_robots.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_pkg_0x0301_inter_among_robots.cpp.i"
	cd /d D:\Omni\Omni\build\HW-Components\devices\referee && D:\ST\STM32CubeCLT_1.16.0\GNU-tools-for-STM32\bin\arm-none-eabi-g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E D:\Omni\Omni\Chassis\HW-Components\devices\referee\src\rfr_pkg_0x0301_inter_among_robots.cpp > CMakeFiles\hwcomponents_devices_referee.dir\src\rfr_pkg_0x0301_inter_among_robots.cpp.i

HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_pkg_0x0301_inter_among_robots.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_pkg_0x0301_inter_among_robots.cpp.s"
	cd /d D:\Omni\Omni\build\HW-Components\devices\referee && D:\ST\STM32CubeCLT_1.16.0\GNU-tools-for-STM32\bin\arm-none-eabi-g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S D:\Omni\Omni\Chassis\HW-Components\devices\referee\src\rfr_pkg_0x0301_inter_among_robots.cpp -o CMakeFiles\hwcomponents_devices_referee.dir\src\rfr_pkg_0x0301_inter_among_robots.cpp.s

# Object files for target hwcomponents_devices_referee
hwcomponents_devices_referee_OBJECTS = \
"CMakeFiles/hwcomponents_devices_referee.dir/src/referee.cpp.obj" \
"CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_crc.cpp.obj" \
"CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_decoder.cpp.obj" \
"CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_encoder.cpp.obj" \
"CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_pkg_0x0301_inter_among_robots.cpp.obj"

# External object files for target hwcomponents_devices_referee
hwcomponents_devices_referee_EXTERNAL_OBJECTS =

D:/Omni/Omni/Chassis/Lib/hwcomponents/devices/referee/libhwcomponents_devices_referee.a: HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/referee.cpp.obj
D:/Omni/Omni/Chassis/Lib/hwcomponents/devices/referee/libhwcomponents_devices_referee.a: HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_crc.cpp.obj
D:/Omni/Omni/Chassis/Lib/hwcomponents/devices/referee/libhwcomponents_devices_referee.a: HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_decoder.cpp.obj
D:/Omni/Omni/Chassis/Lib/hwcomponents/devices/referee/libhwcomponents_devices_referee.a: HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_encoder.cpp.obj
D:/Omni/Omni/Chassis/Lib/hwcomponents/devices/referee/libhwcomponents_devices_referee.a: HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/src/rfr_pkg_0x0301_inter_among_robots.cpp.obj
D:/Omni/Omni/Chassis/Lib/hwcomponents/devices/referee/libhwcomponents_devices_referee.a: HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/build.make
D:/Omni/Omni/Chassis/Lib/hwcomponents/devices/referee/libhwcomponents_devices_referee.a: HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=D:\Omni\Omni\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX static library D:\Omni\Omni\Chassis\Lib\hwcomponents\devices\referee\libhwcomponents_devices_referee.a"
	cd /d D:\Omni\Omni\build\HW-Components\devices\referee && $(CMAKE_COMMAND) -P CMakeFiles\hwcomponents_devices_referee.dir\cmake_clean_target.cmake
	cd /d D:\Omni\Omni\build\HW-Components\devices\referee && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\hwcomponents_devices_referee.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/build: D:/Omni/Omni/Chassis/Lib/hwcomponents/devices/referee/libhwcomponents_devices_referee.a
.PHONY : HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/build

HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/clean:
	cd /d D:\Omni\Omni\build\HW-Components\devices\referee && $(CMAKE_COMMAND) -P CMakeFiles\hwcomponents_devices_referee.dir\cmake_clean.cmake
.PHONY : HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/clean

HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" D:\Omni\Omni\Chassis D:\Omni\Omni\Chassis\HW-Components\devices\referee D:\Omni\Omni\build D:\Omni\Omni\build\HW-Components\devices\referee D:\Omni\Omni\build\HW-Components\devices\referee\CMakeFiles\hwcomponents_devices_referee.dir\DependInfo.cmake "--color=$(COLOR)"
.PHONY : HW-Components/devices/referee/CMakeFiles/hwcomponents_devices_referee.dir/depend

