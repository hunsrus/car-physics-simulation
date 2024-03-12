# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

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

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/gabriel/source-code/bullet3

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/gabriel/source-code/bullet3/build

# Include any dependencies generated for this target.
include src/LinearMath/CMakeFiles/LinearMath.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include src/LinearMath/CMakeFiles/LinearMath.dir/compiler_depend.make

# Include the progress variables for this target.
include src/LinearMath/CMakeFiles/LinearMath.dir/progress.make

# Include the compile flags for this target's objects.
include src/LinearMath/CMakeFiles/LinearMath.dir/flags.make

src/LinearMath/CMakeFiles/LinearMath.dir/btAlignedAllocator.o: src/LinearMath/CMakeFiles/LinearMath.dir/flags.make
src/LinearMath/CMakeFiles/LinearMath.dir/btAlignedAllocator.o: /home/gabriel/source-code/bullet3/src/LinearMath/btAlignedAllocator.cpp
src/LinearMath/CMakeFiles/LinearMath.dir/btAlignedAllocator.o: src/LinearMath/CMakeFiles/LinearMath.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/gabriel/source-code/bullet3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/LinearMath/CMakeFiles/LinearMath.dir/btAlignedAllocator.o"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/LinearMath/CMakeFiles/LinearMath.dir/btAlignedAllocator.o -MF CMakeFiles/LinearMath.dir/btAlignedAllocator.o.d -o CMakeFiles/LinearMath.dir/btAlignedAllocator.o -c /home/gabriel/source-code/bullet3/src/LinearMath/btAlignedAllocator.cpp

src/LinearMath/CMakeFiles/LinearMath.dir/btAlignedAllocator.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/LinearMath.dir/btAlignedAllocator.i"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gabriel/source-code/bullet3/src/LinearMath/btAlignedAllocator.cpp > CMakeFiles/LinearMath.dir/btAlignedAllocator.i

src/LinearMath/CMakeFiles/LinearMath.dir/btAlignedAllocator.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/LinearMath.dir/btAlignedAllocator.s"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gabriel/source-code/bullet3/src/LinearMath/btAlignedAllocator.cpp -o CMakeFiles/LinearMath.dir/btAlignedAllocator.s

src/LinearMath/CMakeFiles/LinearMath.dir/btConvexHull.o: src/LinearMath/CMakeFiles/LinearMath.dir/flags.make
src/LinearMath/CMakeFiles/LinearMath.dir/btConvexHull.o: /home/gabriel/source-code/bullet3/src/LinearMath/btConvexHull.cpp
src/LinearMath/CMakeFiles/LinearMath.dir/btConvexHull.o: src/LinearMath/CMakeFiles/LinearMath.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/gabriel/source-code/bullet3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/LinearMath/CMakeFiles/LinearMath.dir/btConvexHull.o"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/LinearMath/CMakeFiles/LinearMath.dir/btConvexHull.o -MF CMakeFiles/LinearMath.dir/btConvexHull.o.d -o CMakeFiles/LinearMath.dir/btConvexHull.o -c /home/gabriel/source-code/bullet3/src/LinearMath/btConvexHull.cpp

src/LinearMath/CMakeFiles/LinearMath.dir/btConvexHull.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/LinearMath.dir/btConvexHull.i"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gabriel/source-code/bullet3/src/LinearMath/btConvexHull.cpp > CMakeFiles/LinearMath.dir/btConvexHull.i

src/LinearMath/CMakeFiles/LinearMath.dir/btConvexHull.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/LinearMath.dir/btConvexHull.s"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gabriel/source-code/bullet3/src/LinearMath/btConvexHull.cpp -o CMakeFiles/LinearMath.dir/btConvexHull.s

src/LinearMath/CMakeFiles/LinearMath.dir/btConvexHullComputer.o: src/LinearMath/CMakeFiles/LinearMath.dir/flags.make
src/LinearMath/CMakeFiles/LinearMath.dir/btConvexHullComputer.o: /home/gabriel/source-code/bullet3/src/LinearMath/btConvexHullComputer.cpp
src/LinearMath/CMakeFiles/LinearMath.dir/btConvexHullComputer.o: src/LinearMath/CMakeFiles/LinearMath.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/gabriel/source-code/bullet3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/LinearMath/CMakeFiles/LinearMath.dir/btConvexHullComputer.o"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/LinearMath/CMakeFiles/LinearMath.dir/btConvexHullComputer.o -MF CMakeFiles/LinearMath.dir/btConvexHullComputer.o.d -o CMakeFiles/LinearMath.dir/btConvexHullComputer.o -c /home/gabriel/source-code/bullet3/src/LinearMath/btConvexHullComputer.cpp

src/LinearMath/CMakeFiles/LinearMath.dir/btConvexHullComputer.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/LinearMath.dir/btConvexHullComputer.i"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gabriel/source-code/bullet3/src/LinearMath/btConvexHullComputer.cpp > CMakeFiles/LinearMath.dir/btConvexHullComputer.i

src/LinearMath/CMakeFiles/LinearMath.dir/btConvexHullComputer.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/LinearMath.dir/btConvexHullComputer.s"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gabriel/source-code/bullet3/src/LinearMath/btConvexHullComputer.cpp -o CMakeFiles/LinearMath.dir/btConvexHullComputer.s

src/LinearMath/CMakeFiles/LinearMath.dir/btGeometryUtil.o: src/LinearMath/CMakeFiles/LinearMath.dir/flags.make
src/LinearMath/CMakeFiles/LinearMath.dir/btGeometryUtil.o: /home/gabriel/source-code/bullet3/src/LinearMath/btGeometryUtil.cpp
src/LinearMath/CMakeFiles/LinearMath.dir/btGeometryUtil.o: src/LinearMath/CMakeFiles/LinearMath.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/gabriel/source-code/bullet3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/LinearMath/CMakeFiles/LinearMath.dir/btGeometryUtil.o"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/LinearMath/CMakeFiles/LinearMath.dir/btGeometryUtil.o -MF CMakeFiles/LinearMath.dir/btGeometryUtil.o.d -o CMakeFiles/LinearMath.dir/btGeometryUtil.o -c /home/gabriel/source-code/bullet3/src/LinearMath/btGeometryUtil.cpp

src/LinearMath/CMakeFiles/LinearMath.dir/btGeometryUtil.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/LinearMath.dir/btGeometryUtil.i"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gabriel/source-code/bullet3/src/LinearMath/btGeometryUtil.cpp > CMakeFiles/LinearMath.dir/btGeometryUtil.i

src/LinearMath/CMakeFiles/LinearMath.dir/btGeometryUtil.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/LinearMath.dir/btGeometryUtil.s"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gabriel/source-code/bullet3/src/LinearMath/btGeometryUtil.cpp -o CMakeFiles/LinearMath.dir/btGeometryUtil.s

src/LinearMath/CMakeFiles/LinearMath.dir/btPolarDecomposition.o: src/LinearMath/CMakeFiles/LinearMath.dir/flags.make
src/LinearMath/CMakeFiles/LinearMath.dir/btPolarDecomposition.o: /home/gabriel/source-code/bullet3/src/LinearMath/btPolarDecomposition.cpp
src/LinearMath/CMakeFiles/LinearMath.dir/btPolarDecomposition.o: src/LinearMath/CMakeFiles/LinearMath.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/gabriel/source-code/bullet3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object src/LinearMath/CMakeFiles/LinearMath.dir/btPolarDecomposition.o"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/LinearMath/CMakeFiles/LinearMath.dir/btPolarDecomposition.o -MF CMakeFiles/LinearMath.dir/btPolarDecomposition.o.d -o CMakeFiles/LinearMath.dir/btPolarDecomposition.o -c /home/gabriel/source-code/bullet3/src/LinearMath/btPolarDecomposition.cpp

src/LinearMath/CMakeFiles/LinearMath.dir/btPolarDecomposition.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/LinearMath.dir/btPolarDecomposition.i"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gabriel/source-code/bullet3/src/LinearMath/btPolarDecomposition.cpp > CMakeFiles/LinearMath.dir/btPolarDecomposition.i

src/LinearMath/CMakeFiles/LinearMath.dir/btPolarDecomposition.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/LinearMath.dir/btPolarDecomposition.s"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gabriel/source-code/bullet3/src/LinearMath/btPolarDecomposition.cpp -o CMakeFiles/LinearMath.dir/btPolarDecomposition.s

src/LinearMath/CMakeFiles/LinearMath.dir/btQuickprof.o: src/LinearMath/CMakeFiles/LinearMath.dir/flags.make
src/LinearMath/CMakeFiles/LinearMath.dir/btQuickprof.o: /home/gabriel/source-code/bullet3/src/LinearMath/btQuickprof.cpp
src/LinearMath/CMakeFiles/LinearMath.dir/btQuickprof.o: src/LinearMath/CMakeFiles/LinearMath.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/gabriel/source-code/bullet3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object src/LinearMath/CMakeFiles/LinearMath.dir/btQuickprof.o"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/LinearMath/CMakeFiles/LinearMath.dir/btQuickprof.o -MF CMakeFiles/LinearMath.dir/btQuickprof.o.d -o CMakeFiles/LinearMath.dir/btQuickprof.o -c /home/gabriel/source-code/bullet3/src/LinearMath/btQuickprof.cpp

src/LinearMath/CMakeFiles/LinearMath.dir/btQuickprof.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/LinearMath.dir/btQuickprof.i"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gabriel/source-code/bullet3/src/LinearMath/btQuickprof.cpp > CMakeFiles/LinearMath.dir/btQuickprof.i

src/LinearMath/CMakeFiles/LinearMath.dir/btQuickprof.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/LinearMath.dir/btQuickprof.s"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gabriel/source-code/bullet3/src/LinearMath/btQuickprof.cpp -o CMakeFiles/LinearMath.dir/btQuickprof.s

src/LinearMath/CMakeFiles/LinearMath.dir/btReducedVector.o: src/LinearMath/CMakeFiles/LinearMath.dir/flags.make
src/LinearMath/CMakeFiles/LinearMath.dir/btReducedVector.o: /home/gabriel/source-code/bullet3/src/LinearMath/btReducedVector.cpp
src/LinearMath/CMakeFiles/LinearMath.dir/btReducedVector.o: src/LinearMath/CMakeFiles/LinearMath.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/gabriel/source-code/bullet3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object src/LinearMath/CMakeFiles/LinearMath.dir/btReducedVector.o"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/LinearMath/CMakeFiles/LinearMath.dir/btReducedVector.o -MF CMakeFiles/LinearMath.dir/btReducedVector.o.d -o CMakeFiles/LinearMath.dir/btReducedVector.o -c /home/gabriel/source-code/bullet3/src/LinearMath/btReducedVector.cpp

src/LinearMath/CMakeFiles/LinearMath.dir/btReducedVector.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/LinearMath.dir/btReducedVector.i"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gabriel/source-code/bullet3/src/LinearMath/btReducedVector.cpp > CMakeFiles/LinearMath.dir/btReducedVector.i

src/LinearMath/CMakeFiles/LinearMath.dir/btReducedVector.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/LinearMath.dir/btReducedVector.s"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gabriel/source-code/bullet3/src/LinearMath/btReducedVector.cpp -o CMakeFiles/LinearMath.dir/btReducedVector.s

src/LinearMath/CMakeFiles/LinearMath.dir/btSerializer.o: src/LinearMath/CMakeFiles/LinearMath.dir/flags.make
src/LinearMath/CMakeFiles/LinearMath.dir/btSerializer.o: /home/gabriel/source-code/bullet3/src/LinearMath/btSerializer.cpp
src/LinearMath/CMakeFiles/LinearMath.dir/btSerializer.o: src/LinearMath/CMakeFiles/LinearMath.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/gabriel/source-code/bullet3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object src/LinearMath/CMakeFiles/LinearMath.dir/btSerializer.o"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/LinearMath/CMakeFiles/LinearMath.dir/btSerializer.o -MF CMakeFiles/LinearMath.dir/btSerializer.o.d -o CMakeFiles/LinearMath.dir/btSerializer.o -c /home/gabriel/source-code/bullet3/src/LinearMath/btSerializer.cpp

src/LinearMath/CMakeFiles/LinearMath.dir/btSerializer.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/LinearMath.dir/btSerializer.i"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gabriel/source-code/bullet3/src/LinearMath/btSerializer.cpp > CMakeFiles/LinearMath.dir/btSerializer.i

src/LinearMath/CMakeFiles/LinearMath.dir/btSerializer.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/LinearMath.dir/btSerializer.s"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gabriel/source-code/bullet3/src/LinearMath/btSerializer.cpp -o CMakeFiles/LinearMath.dir/btSerializer.s

src/LinearMath/CMakeFiles/LinearMath.dir/btSerializer64.o: src/LinearMath/CMakeFiles/LinearMath.dir/flags.make
src/LinearMath/CMakeFiles/LinearMath.dir/btSerializer64.o: /home/gabriel/source-code/bullet3/src/LinearMath/btSerializer64.cpp
src/LinearMath/CMakeFiles/LinearMath.dir/btSerializer64.o: src/LinearMath/CMakeFiles/LinearMath.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/gabriel/source-code/bullet3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object src/LinearMath/CMakeFiles/LinearMath.dir/btSerializer64.o"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/LinearMath/CMakeFiles/LinearMath.dir/btSerializer64.o -MF CMakeFiles/LinearMath.dir/btSerializer64.o.d -o CMakeFiles/LinearMath.dir/btSerializer64.o -c /home/gabriel/source-code/bullet3/src/LinearMath/btSerializer64.cpp

src/LinearMath/CMakeFiles/LinearMath.dir/btSerializer64.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/LinearMath.dir/btSerializer64.i"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gabriel/source-code/bullet3/src/LinearMath/btSerializer64.cpp > CMakeFiles/LinearMath.dir/btSerializer64.i

src/LinearMath/CMakeFiles/LinearMath.dir/btSerializer64.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/LinearMath.dir/btSerializer64.s"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gabriel/source-code/bullet3/src/LinearMath/btSerializer64.cpp -o CMakeFiles/LinearMath.dir/btSerializer64.s

src/LinearMath/CMakeFiles/LinearMath.dir/btThreads.o: src/LinearMath/CMakeFiles/LinearMath.dir/flags.make
src/LinearMath/CMakeFiles/LinearMath.dir/btThreads.o: /home/gabriel/source-code/bullet3/src/LinearMath/btThreads.cpp
src/LinearMath/CMakeFiles/LinearMath.dir/btThreads.o: src/LinearMath/CMakeFiles/LinearMath.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/gabriel/source-code/bullet3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object src/LinearMath/CMakeFiles/LinearMath.dir/btThreads.o"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/LinearMath/CMakeFiles/LinearMath.dir/btThreads.o -MF CMakeFiles/LinearMath.dir/btThreads.o.d -o CMakeFiles/LinearMath.dir/btThreads.o -c /home/gabriel/source-code/bullet3/src/LinearMath/btThreads.cpp

src/LinearMath/CMakeFiles/LinearMath.dir/btThreads.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/LinearMath.dir/btThreads.i"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gabriel/source-code/bullet3/src/LinearMath/btThreads.cpp > CMakeFiles/LinearMath.dir/btThreads.i

src/LinearMath/CMakeFiles/LinearMath.dir/btThreads.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/LinearMath.dir/btThreads.s"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gabriel/source-code/bullet3/src/LinearMath/btThreads.cpp -o CMakeFiles/LinearMath.dir/btThreads.s

src/LinearMath/CMakeFiles/LinearMath.dir/btVector3.o: src/LinearMath/CMakeFiles/LinearMath.dir/flags.make
src/LinearMath/CMakeFiles/LinearMath.dir/btVector3.o: /home/gabriel/source-code/bullet3/src/LinearMath/btVector3.cpp
src/LinearMath/CMakeFiles/LinearMath.dir/btVector3.o: src/LinearMath/CMakeFiles/LinearMath.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/gabriel/source-code/bullet3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object src/LinearMath/CMakeFiles/LinearMath.dir/btVector3.o"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/LinearMath/CMakeFiles/LinearMath.dir/btVector3.o -MF CMakeFiles/LinearMath.dir/btVector3.o.d -o CMakeFiles/LinearMath.dir/btVector3.o -c /home/gabriel/source-code/bullet3/src/LinearMath/btVector3.cpp

src/LinearMath/CMakeFiles/LinearMath.dir/btVector3.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/LinearMath.dir/btVector3.i"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gabriel/source-code/bullet3/src/LinearMath/btVector3.cpp > CMakeFiles/LinearMath.dir/btVector3.i

src/LinearMath/CMakeFiles/LinearMath.dir/btVector3.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/LinearMath.dir/btVector3.s"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gabriel/source-code/bullet3/src/LinearMath/btVector3.cpp -o CMakeFiles/LinearMath.dir/btVector3.s

src/LinearMath/CMakeFiles/LinearMath.dir/TaskScheduler/btTaskScheduler.o: src/LinearMath/CMakeFiles/LinearMath.dir/flags.make
src/LinearMath/CMakeFiles/LinearMath.dir/TaskScheduler/btTaskScheduler.o: /home/gabriel/source-code/bullet3/src/LinearMath/TaskScheduler/btTaskScheduler.cpp
src/LinearMath/CMakeFiles/LinearMath.dir/TaskScheduler/btTaskScheduler.o: src/LinearMath/CMakeFiles/LinearMath.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/gabriel/source-code/bullet3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object src/LinearMath/CMakeFiles/LinearMath.dir/TaskScheduler/btTaskScheduler.o"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/LinearMath/CMakeFiles/LinearMath.dir/TaskScheduler/btTaskScheduler.o -MF CMakeFiles/LinearMath.dir/TaskScheduler/btTaskScheduler.o.d -o CMakeFiles/LinearMath.dir/TaskScheduler/btTaskScheduler.o -c /home/gabriel/source-code/bullet3/src/LinearMath/TaskScheduler/btTaskScheduler.cpp

src/LinearMath/CMakeFiles/LinearMath.dir/TaskScheduler/btTaskScheduler.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/LinearMath.dir/TaskScheduler/btTaskScheduler.i"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gabriel/source-code/bullet3/src/LinearMath/TaskScheduler/btTaskScheduler.cpp > CMakeFiles/LinearMath.dir/TaskScheduler/btTaskScheduler.i

src/LinearMath/CMakeFiles/LinearMath.dir/TaskScheduler/btTaskScheduler.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/LinearMath.dir/TaskScheduler/btTaskScheduler.s"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gabriel/source-code/bullet3/src/LinearMath/TaskScheduler/btTaskScheduler.cpp -o CMakeFiles/LinearMath.dir/TaskScheduler/btTaskScheduler.s

src/LinearMath/CMakeFiles/LinearMath.dir/TaskScheduler/btThreadSupportPosix.o: src/LinearMath/CMakeFiles/LinearMath.dir/flags.make
src/LinearMath/CMakeFiles/LinearMath.dir/TaskScheduler/btThreadSupportPosix.o: /home/gabriel/source-code/bullet3/src/LinearMath/TaskScheduler/btThreadSupportPosix.cpp
src/LinearMath/CMakeFiles/LinearMath.dir/TaskScheduler/btThreadSupportPosix.o: src/LinearMath/CMakeFiles/LinearMath.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/gabriel/source-code/bullet3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building CXX object src/LinearMath/CMakeFiles/LinearMath.dir/TaskScheduler/btThreadSupportPosix.o"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/LinearMath/CMakeFiles/LinearMath.dir/TaskScheduler/btThreadSupportPosix.o -MF CMakeFiles/LinearMath.dir/TaskScheduler/btThreadSupportPosix.o.d -o CMakeFiles/LinearMath.dir/TaskScheduler/btThreadSupportPosix.o -c /home/gabriel/source-code/bullet3/src/LinearMath/TaskScheduler/btThreadSupportPosix.cpp

src/LinearMath/CMakeFiles/LinearMath.dir/TaskScheduler/btThreadSupportPosix.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/LinearMath.dir/TaskScheduler/btThreadSupportPosix.i"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gabriel/source-code/bullet3/src/LinearMath/TaskScheduler/btThreadSupportPosix.cpp > CMakeFiles/LinearMath.dir/TaskScheduler/btThreadSupportPosix.i

src/LinearMath/CMakeFiles/LinearMath.dir/TaskScheduler/btThreadSupportPosix.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/LinearMath.dir/TaskScheduler/btThreadSupportPosix.s"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gabriel/source-code/bullet3/src/LinearMath/TaskScheduler/btThreadSupportPosix.cpp -o CMakeFiles/LinearMath.dir/TaskScheduler/btThreadSupportPosix.s

src/LinearMath/CMakeFiles/LinearMath.dir/TaskScheduler/btThreadSupportWin32.o: src/LinearMath/CMakeFiles/LinearMath.dir/flags.make
src/LinearMath/CMakeFiles/LinearMath.dir/TaskScheduler/btThreadSupportWin32.o: /home/gabriel/source-code/bullet3/src/LinearMath/TaskScheduler/btThreadSupportWin32.cpp
src/LinearMath/CMakeFiles/LinearMath.dir/TaskScheduler/btThreadSupportWin32.o: src/LinearMath/CMakeFiles/LinearMath.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/gabriel/source-code/bullet3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building CXX object src/LinearMath/CMakeFiles/LinearMath.dir/TaskScheduler/btThreadSupportWin32.o"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/LinearMath/CMakeFiles/LinearMath.dir/TaskScheduler/btThreadSupportWin32.o -MF CMakeFiles/LinearMath.dir/TaskScheduler/btThreadSupportWin32.o.d -o CMakeFiles/LinearMath.dir/TaskScheduler/btThreadSupportWin32.o -c /home/gabriel/source-code/bullet3/src/LinearMath/TaskScheduler/btThreadSupportWin32.cpp

src/LinearMath/CMakeFiles/LinearMath.dir/TaskScheduler/btThreadSupportWin32.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/LinearMath.dir/TaskScheduler/btThreadSupportWin32.i"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gabriel/source-code/bullet3/src/LinearMath/TaskScheduler/btThreadSupportWin32.cpp > CMakeFiles/LinearMath.dir/TaskScheduler/btThreadSupportWin32.i

src/LinearMath/CMakeFiles/LinearMath.dir/TaskScheduler/btThreadSupportWin32.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/LinearMath.dir/TaskScheduler/btThreadSupportWin32.s"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gabriel/source-code/bullet3/src/LinearMath/TaskScheduler/btThreadSupportWin32.cpp -o CMakeFiles/LinearMath.dir/TaskScheduler/btThreadSupportWin32.s

# Object files for target LinearMath
LinearMath_OBJECTS = \
"CMakeFiles/LinearMath.dir/btAlignedAllocator.o" \
"CMakeFiles/LinearMath.dir/btConvexHull.o" \
"CMakeFiles/LinearMath.dir/btConvexHullComputer.o" \
"CMakeFiles/LinearMath.dir/btGeometryUtil.o" \
"CMakeFiles/LinearMath.dir/btPolarDecomposition.o" \
"CMakeFiles/LinearMath.dir/btQuickprof.o" \
"CMakeFiles/LinearMath.dir/btReducedVector.o" \
"CMakeFiles/LinearMath.dir/btSerializer.o" \
"CMakeFiles/LinearMath.dir/btSerializer64.o" \
"CMakeFiles/LinearMath.dir/btThreads.o" \
"CMakeFiles/LinearMath.dir/btVector3.o" \
"CMakeFiles/LinearMath.dir/TaskScheduler/btTaskScheduler.o" \
"CMakeFiles/LinearMath.dir/TaskScheduler/btThreadSupportPosix.o" \
"CMakeFiles/LinearMath.dir/TaskScheduler/btThreadSupportWin32.o"

# External object files for target LinearMath
LinearMath_EXTERNAL_OBJECTS =

src/LinearMath/libLinearMath.a: src/LinearMath/CMakeFiles/LinearMath.dir/btAlignedAllocator.o
src/LinearMath/libLinearMath.a: src/LinearMath/CMakeFiles/LinearMath.dir/btConvexHull.o
src/LinearMath/libLinearMath.a: src/LinearMath/CMakeFiles/LinearMath.dir/btConvexHullComputer.o
src/LinearMath/libLinearMath.a: src/LinearMath/CMakeFiles/LinearMath.dir/btGeometryUtil.o
src/LinearMath/libLinearMath.a: src/LinearMath/CMakeFiles/LinearMath.dir/btPolarDecomposition.o
src/LinearMath/libLinearMath.a: src/LinearMath/CMakeFiles/LinearMath.dir/btQuickprof.o
src/LinearMath/libLinearMath.a: src/LinearMath/CMakeFiles/LinearMath.dir/btReducedVector.o
src/LinearMath/libLinearMath.a: src/LinearMath/CMakeFiles/LinearMath.dir/btSerializer.o
src/LinearMath/libLinearMath.a: src/LinearMath/CMakeFiles/LinearMath.dir/btSerializer64.o
src/LinearMath/libLinearMath.a: src/LinearMath/CMakeFiles/LinearMath.dir/btThreads.o
src/LinearMath/libLinearMath.a: src/LinearMath/CMakeFiles/LinearMath.dir/btVector3.o
src/LinearMath/libLinearMath.a: src/LinearMath/CMakeFiles/LinearMath.dir/TaskScheduler/btTaskScheduler.o
src/LinearMath/libLinearMath.a: src/LinearMath/CMakeFiles/LinearMath.dir/TaskScheduler/btThreadSupportPosix.o
src/LinearMath/libLinearMath.a: src/LinearMath/CMakeFiles/LinearMath.dir/TaskScheduler/btThreadSupportWin32.o
src/LinearMath/libLinearMath.a: src/LinearMath/CMakeFiles/LinearMath.dir/build.make
src/LinearMath/libLinearMath.a: src/LinearMath/CMakeFiles/LinearMath.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/gabriel/source-code/bullet3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Linking CXX static library libLinearMath.a"
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && $(CMAKE_COMMAND) -P CMakeFiles/LinearMath.dir/cmake_clean_target.cmake
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/LinearMath.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/LinearMath/CMakeFiles/LinearMath.dir/build: src/LinearMath/libLinearMath.a
.PHONY : src/LinearMath/CMakeFiles/LinearMath.dir/build

src/LinearMath/CMakeFiles/LinearMath.dir/clean:
	cd /home/gabriel/source-code/bullet3/build/src/LinearMath && $(CMAKE_COMMAND) -P CMakeFiles/LinearMath.dir/cmake_clean.cmake
.PHONY : src/LinearMath/CMakeFiles/LinearMath.dir/clean

src/LinearMath/CMakeFiles/LinearMath.dir/depend:
	cd /home/gabriel/source-code/bullet3/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/gabriel/source-code/bullet3 /home/gabriel/source-code/bullet3/src/LinearMath /home/gabriel/source-code/bullet3/build /home/gabriel/source-code/bullet3/build/src/LinearMath /home/gabriel/source-code/bullet3/build/src/LinearMath/CMakeFiles/LinearMath.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : src/LinearMath/CMakeFiles/LinearMath.dir/depend
