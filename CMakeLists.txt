cmake_minimum_required(VERSION 3.10)
project(miniroscpp)

#find_package(Boost REQUIRED COMPONENTS system thread)

# TODO: Get rid of console_bridge
find_package(console_bridge REQUIRED)

find_package(catkin REQUIRED)

catkin_package(
  DEPENDS console_bridge
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME})

include(CheckIncludeFile)
include(CheckFunctionExists)
include(CheckCXXSourceCompiles)

include_directories(include ${console_bridge_INCLUDE_DIRS})

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

#
SET(cpp17_common_SRC src/debug.cpp src/header.cpp)
SET(serialization_SRC src/serialization.cpp)
SET(rostime_SRC
	src/rostime/duration.cpp
	src/rostime/rate.cpp
	src/rostime/time.cpp)

add_library(miniroscpp ${cpp17_common_SRC} ${serialization_SRC} ${rostime_SRC})

set_property(TARGET miniroscpp PROPERTY CXX_STANDARD 17)
target_link_libraries(miniroscpp ${console_bridge_LIBRARIES})

install(TARGETS miniroscpp
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
