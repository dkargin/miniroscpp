cmake_minimum_required(VERSION 3.10)
project(miniroscpp)

find_package(catkin QUIET)

if(catkin_FOUND)
catkin_package(
  DEPENDS
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME})
else(catkin_FOUND)

endif(catkin_FOUND)

include(CheckIncludeFile)
include(CheckFunctionExists)
include(CheckCXXSourceCompiles)

include_directories(include)
include_directories(include/generated)

get_directory_property(hasParent PARENT_DIRECTORY)
if(hasParent)
  message(STATUS "miniroscpp was added as a submodule.")
  set(MINIROS_SUBMODULE TRUE)
else()
  set(MINIROS_SUBMODULE FALSE)
endif()

option(MINIROS_USE_SYSTEM_BZIP2 "Use bzip2 from system" OFF)
option(MINIROS_USE_SYSTEM_LZ4 "Use lz4 from system" OFF)

option(MINIROS_BUILD_TESTS "Build tests" OFF)

# execinfo.h is needed for backtrace on glibc systems
CHECK_INCLUDE_FILE(execinfo.h HAVE_EXECINFO_H)
if(HAVE_EXECINFO_H)
  add_definitions(-DHAVE_EXECINFO_H=1)
endif(HAVE_EXECINFO_H)
# do we have demangle capability?
# CHECK_INCLUDE_FILE doesn't work here for some reason
CHECK_CXX_SOURCE_COMPILES("#include<cxxabi.h>\nint main(int argc,char**argv){}" HAVE_CXXABI_H)
if(HAVE_CXXABI_H)
  add_definitions(-DHAVE_CXXABI_H=1)
endif()

CHECK_FUNCTION_EXISTS(backtrace HAVE_GLIBC_BACKTRACE)
if(HAVE_GLIBC_BACKTRACE)
  add_definitions(-DHAVE_GLIBC_BACKTRACE)
endif(HAVE_GLIBC_BACKTRACE)

# Processing external modules like bzip2 or lz4
add_subdirectory(external)

add_subdirectory(src)

if (MINIROS_BUILD_TESTS)
	find_package(GTEST QUIET)
	if (GTEST_FOUND)
		add_subdirectory(test)
	else()
		message(STATUS "Failed to find gtest. No tests will be built")
	endif()
endif()

if(catkin_FOUND)
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
endif(catkin_FOUND)
