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
CMAKE_SOURCE_DIR = "/mnt/c/Users/ajosh/Desktop/TextBooks^CMU/ROBOTICS/Act/EMSD(24-671)/ROM_Rehab_Control/ROM_Rehab_Control/src/RomBot/test"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/mnt/c/Users/ajosh/Desktop/TextBooks^CMU/ROBOTICS/Act/EMSD(24-671)/ROM_Rehab_Control/ROM_Rehab_Control/src/RomBot"

# Include any dependencies generated for this target.
include CMakeFiles/RomBotTests.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/RomBotTests.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/RomBotTests.dir/flags.make

CMakeFiles/RomBotTests.dir/RomBotTests.cpp.o: CMakeFiles/RomBotTests.dir/flags.make
CMakeFiles/RomBotTests.dir/RomBotTests.cpp.o: test/RomBotTests.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/mnt/c/Users/ajosh/Desktop/TextBooks^CMU/ROBOTICS/Act/EMSD(24-671)/ROM_Rehab_Control/ROM_Rehab_Control/src/RomBot/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/RomBotTests.dir/RomBotTests.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/RomBotTests.dir/RomBotTests.cpp.o -c "/mnt/c/Users/ajosh/Desktop/TextBooks^CMU/ROBOTICS/Act/EMSD(24-671)/ROM_Rehab_Control/ROM_Rehab_Control/src/RomBot/test/RomBotTests.cpp"

CMakeFiles/RomBotTests.dir/RomBotTests.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RomBotTests.dir/RomBotTests.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/mnt/c/Users/ajosh/Desktop/TextBooks^CMU/ROBOTICS/Act/EMSD(24-671)/ROM_Rehab_Control/ROM_Rehab_Control/src/RomBot/test/RomBotTests.cpp" > CMakeFiles/RomBotTests.dir/RomBotTests.cpp.i

CMakeFiles/RomBotTests.dir/RomBotTests.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RomBotTests.dir/RomBotTests.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/mnt/c/Users/ajosh/Desktop/TextBooks^CMU/ROBOTICS/Act/EMSD(24-671)/ROM_Rehab_Control/ROM_Rehab_Control/src/RomBot/test/RomBotTests.cpp" -o CMakeFiles/RomBotTests.dir/RomBotTests.cpp.s

CMakeFiles/RomBotTests.dir/RomBotTests.cpp.o.requires:

.PHONY : CMakeFiles/RomBotTests.dir/RomBotTests.cpp.o.requires

CMakeFiles/RomBotTests.dir/RomBotTests.cpp.o.provides: CMakeFiles/RomBotTests.dir/RomBotTests.cpp.o.requires
	$(MAKE) -f CMakeFiles/RomBotTests.dir/build.make CMakeFiles/RomBotTests.dir/RomBotTests.cpp.o.provides.build
.PHONY : CMakeFiles/RomBotTests.dir/RomBotTests.cpp.o.provides

CMakeFiles/RomBotTests.dir/RomBotTests.cpp.o.provides.build: CMakeFiles/RomBotTests.dir/RomBotTests.cpp.o


# Object files for target RomBotTests
RomBotTests_OBJECTS = \
"CMakeFiles/RomBotTests.dir/RomBotTests.cpp.o"

# External object files for target RomBotTests
RomBotTests_EXTERNAL_OBJECTS =

RomBotTests: CMakeFiles/RomBotTests.dir/RomBotTests.cpp.o
RomBotTests: CMakeFiles/RomBotTests.dir/build.make
RomBotTests: CMakeFiles/RomBotTests.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/mnt/c/Users/ajosh/Desktop/TextBooks^CMU/ROBOTICS/Act/EMSD(24-671)/ROM_Rehab_Control/ROM_Rehab_Control/src/RomBot/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable RomBotTests"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/RomBotTests.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/RomBotTests.dir/build: RomBotTests

.PHONY : CMakeFiles/RomBotTests.dir/build

CMakeFiles/RomBotTests.dir/requires: CMakeFiles/RomBotTests.dir/RomBotTests.cpp.o.requires

.PHONY : CMakeFiles/RomBotTests.dir/requires

CMakeFiles/RomBotTests.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/RomBotTests.dir/cmake_clean.cmake
.PHONY : CMakeFiles/RomBotTests.dir/clean

CMakeFiles/RomBotTests.dir/depend:
	cd "/mnt/c/Users/ajosh/Desktop/TextBooks^CMU/ROBOTICS/Act/EMSD(24-671)/ROM_Rehab_Control/ROM_Rehab_Control/src/RomBot" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/mnt/c/Users/ajosh/Desktop/TextBooks^CMU/ROBOTICS/Act/EMSD(24-671)/ROM_Rehab_Control/ROM_Rehab_Control/src/RomBot/test" "/mnt/c/Users/ajosh/Desktop/TextBooks^CMU/ROBOTICS/Act/EMSD(24-671)/ROM_Rehab_Control/ROM_Rehab_Control/src/RomBot/test" "/mnt/c/Users/ajosh/Desktop/TextBooks^CMU/ROBOTICS/Act/EMSD(24-671)/ROM_Rehab_Control/ROM_Rehab_Control/src/RomBot" "/mnt/c/Users/ajosh/Desktop/TextBooks^CMU/ROBOTICS/Act/EMSD(24-671)/ROM_Rehab_Control/ROM_Rehab_Control/src/RomBot" "/mnt/c/Users/ajosh/Desktop/TextBooks^CMU/ROBOTICS/Act/EMSD(24-671)/ROM_Rehab_Control/ROM_Rehab_Control/src/RomBot/CMakeFiles/RomBotTests.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/RomBotTests.dir/depend

