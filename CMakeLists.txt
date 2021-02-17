# We need support for https://cmake.org/cmake/help/git-stage/policy/CMP0077.htmlm which was introduced in 3.13
cmake_minimum_required(VERSION 3.13)

# Setting a policy to properly override 'option' variables. 
# We need it to enforce flags to bzip2
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
cmake_policy(SET CMP0077 NEW)

project(roscxx VERSION 0.1.0)


include(CheckIncludeFile)
include(CheckFunctionExists)
include(CheckCXXSourceCompiles)
include(CMakePackageConfigHelpers)


find_package(catkin QUIET)

if(catkin_FOUND)
  catkin_package(
    DEPENDS
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME})
endif(catkin_FOUND)


set(MINIROS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
	CACHE PATH "Path to MiniROS include dirs")
set(MINIROS_INCLUDE_GENERATED_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/generated
	CACHE PATH "Path to MiniROS generated messages")

get_directory_property(hasParent PARENT_DIRECTORY)
if(hasParent)
  message(STATUS "miniroscpp was added as a submodule.")
  set(ROSCXX_SUBMODULE TRUE)
else()
  set(ROSCXX_SUBMODULE FALSE)
endif()

set(MINIROS_ADDITIONAL_COMPONENTS $<NOT:${ROSCXX_SUBMODULE}>)

option(ROSCXX_USE_SYSTEM_BZIP2 "Use bzip2 from system" OFF)
option(ROSCXX_USE_SYSTEM_LZ4 "Use lz4 from system" OFF)
option(ROSCXX_BUILD_EXAMPLES "Build examples" ${MINIROS_ADDITIONAL_COMPONENTS})
option(ROSCXX_BUILD_TESTS "Build tests" ${MINIROS_ADDITIONAL_COMPONENTS})

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
set(EXTERNAL_INCLUDE "")

add_subdirectory(external)
add_subdirectory(src)

if (MINIROS_BUILD_EXAMPLES)
	add_subdirectory(examples)
endif ()

if (MINIROS_BUILD_TESTS)
	find_package(GTEST QUIET)
	if (GTEST_FOUND)
		add_subdirectory(test)
	else()
		message(STATUS "Failed to find gtest. No tests will be built")
	endif()
endif()

if(catkin_FOUND)
  # Catkin does all the packaging by itself. Also we do not need to install pregenerated messages,
  # since we are able to run gencxx directly here.
  install(DIRECTORY include/
    DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h")
else()
  # Some workaround for adding include directores from different folders
  target_include_directories(minibag_storage PUBLIC include)
  target_include_directories(miniroscpp PUBLIC include)

  # ... for the build tree
  set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
  configure_file(roscxxConfig.cmake.in "${PROJECT_BINARY_DIR}/roscxxConfig.cmake" @ONLY)

  # ... for the install tree
  set(CONF_INCLUDE_DIRS "\${roscxx_CMAKE_DIR}/${REL_INCLUDE_DIR}")
  configure_file(roscxxConfig.cmake.in
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/roscxxConfig.cmake" @ONLY)

  # set_property(TARGET MiniROSTargets PROPERTY VERSION ${PROJECT_VERSION})  
  # generate the version file for the config file
  write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/roscxxConfigVersion.cmake"
    VERSION "${PROJECT_VERSION}"
    COMPATIBILITY AnyNewerVersion
  )
  
  # Installing regular headers
  install(DIRECTORY include DESTINATION include FILES_MATCHING PATTERN "*.h")

  # Installing generated headers for message types
  install(DIRECTORY include/generated DESTINATION include FILES_MATCHING PATTERN "*.hxx")

  install(FILES
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/roscxxConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/roscxxConfigVersion.cmake"
    DESTINATION lib/cmake/roscxx
  )

  #export(EXPORT MiniROSTargets FILE "${CMAKE_CURRENT_BINARY_DIR}/cmake/roscxxTargets.cmake")
  # TODO: Install generated targets
  #install(EXPORT MiniROSTargets
  #  FILE roscxxTargets.cmake
  #  NAMESPACE roscxx::
  #  DESTINATION lib/cmake/roscxx
  #)
endif(catkin_FOUND)
