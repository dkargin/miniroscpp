# We need support for https://cmake.org/cmake/help/git-stage/policy/CMP0077.htmlm which was introduced in 3.13
cmake_minimum_required(VERSION 3.13)

# Setting a policy to properly override 'option' variables. 
# We need it to enforce flags to bzip2
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
cmake_policy(SET CMP0077 NEW)

project(miniros VERSION 0.5.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(CheckIncludeFile)
include(CheckIncludeFiles)
include(CheckFunctionExists)
include(CheckCXXSourceCompiles)
include(CheckCXXSymbolExists)
include(CMakePackageConfigHelpers)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

option(MINIROS_USE_CATKIN "Use ros1 catkin build system" OFF)
option(MINIROS_USE_SYSTEM_BZIP2 "Use bzip2 from system" OFF)
option(MINIROS_USE_SYSTEM_LZ4 "Use lz4 from system" OFF)
option(MINIROS_USE_SYSTEM_PROGRAM_OPTIONS "Use boost::program_options instead of local implementation" OFF)
option(MINIROS_BUILD_EXAMPLES "Build examples" ${MINIROS_ADDITIONAL_COMPONENTS})
option(MINIROS_BUILD_ROSBAG_APPS "Build rosbag applications, like rosbag/play and rosbag/record" OFF)
option(MINIROS_BUILD_TESTS "Build tests" ${MINIROS_ADDITIONAL_COMPONENTS})
option(MINIROS_BUILD_SHARED_LIBS "Build shared version of miniros" ON)
option(MINIROS_ROSBAG_USE_GPGME_ENCRYPTION "Use GPGME encryption support" OFF)
option(MINIROS_ROSBAG_USE_OPENSSL_ENCRYPTION "Use openssl encryption for rosbag" OFF)
option(MINIROS_USE_LIBSYSTEMD "Signal systemd when node has actually started" ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

include_directories(include)

set(MINIROS_INCLUDE_DIRS ${CMAKE_CURRENT_BINARY_DIR})
set(MINIROS_XMLRPCPP_B64_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/xmlrpcpp/libb64/include")

set(CMAKE_DEBUG_POSTFIX d)

set(MINIROS_INCLUDE_GENERATED_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/generated
    CACHE PATH "Path to MiniROS generated messages")

# Root include path for helper libraries like xmlrpc
set(MINIROS_INCLUDE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/include/miniros)

set(MINIROS_INCLUDE_TRANSPORT ${CMAKE_CURRENT_SOURCE_DIR}/include/transport)

# Checking if miniros is added as a submodule
get_directory_property(hasParent PARENT_DIRECTORY)
if(hasParent)
  message(STATUS "${PROJECT_NAME} was added as a submodule.")
  set(MINIROS_SUBMODULE TRUE)
else()
  set(MINIROS_SUBMODULE FALSE)
  set_property(GLOBAL PROPERTY USE_FOLDERS ON)
endif()

set(MINIROS_ADDITIONAL_COMPONENTS $<NOT:${MINIROS_SUBMODULE}>)

if (MINIROS_USE_CATKIN)
  find_package(catkin QUIET)
  if(catkin_FOUND)
    message(STATUS "Miniros has been used with catkin")
    catkin_package(
      DEPENDS
      INCLUDE_DIRS include include/generated
      LIBRARIES roscxx bag_storage)
  else()
    message(STATUS "Catkin was not found")
  endif(catkin_FOUND)
endif()


# Generate external config
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/include/miniros/config.h.in" "miniros/config.h")

# Processing external modules like bzip2 or lz4
set(EXTERNAL_INCLUDE "")

if (WIN32)
  # We have a lot of unix code here, and MSVC is always angry with these methods
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()

set(XMLRPC_SRC_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/src/xmlrpc)

find_package(Threads REQUIRED)

add_subdirectory(external)
add_subdirectory(src)

if (catkin_FOUND)
  # Catkin does all the packaging by itself. Also we do not need to install pregenerated messages,
  # since we are able to run gencxx directly here.
  install(DIRECTORY include/
    DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h")
  install(TARGETS ${MINIROS_EXPORT} EXPORT ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )
else()
  # Some workaround for adding include directores from different folders
  target_include_directories(bag_storage PUBLIC 
    $<BUILD_INTERFACE:${MINIROS_INCLUDE_DIRS}>
    $<INSTALL_INTERFACE:include>
  )

  target_include_directories(roscxx PUBLIC
    $<BUILD_INTERFACE:${MINIROS_INCLUDE_DIRS}>
    $<INSTALL_INTERFACE:include>
  )
  # ... for the build tree
  set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
  set(GENERATED_INCLUDE "generated")
  configure_file(${PROJECT_NAME}Config.cmake.in "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" @ONLY)

  # ... for the install tree
  set(CONF_INCLUDE_DIRS "\${roscxx_CMAKE_DIR}/${REL_INCLUDE_DIR}")
  set(GENERATED_INCLUDE "${PROJECT_NAME}/generated")
  set(MINIROS_GENERATED_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include/generated")
  configure_file(${PROJECT_NAME}Config.cmake.in
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake" @ONLY)

  # set_property(TARGET MiniROSTargets PROPERTY VERSION ${PROJECT_VERSION})  
  # generate the version file for the config file
  write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    VERSION "${PROJECT_VERSION}"
    COMPATIBILITY AnyNewerVersion
  )
  
  # Installing regular headers
  install(DIRECTORY include/ DESTINATION include FILES_MATCHING PATTERN "*.h")
  install(DIRECTORY include/ DESTINATION include FILES_MATCHING PATTERN "*.hxx")

  install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/miniros/config.h 
    DESTINATION include/miniros/)

  # Installing generated headers for message types
  install(DIRECTORY include/generated DESTINATION include/${PROJECT_NAME} FILES_MATCHING PATTERN "*.hxx")
  install(DIRECTORY include/generated DESTINATION include/${PROJECT_NAME} FILES_MATCHING PATTERN "*.h")

  install(FILES
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION lib/cmake/${PROJECT_NAME}
  )

  install(TARGETS ${MINIROS_EXPORT} EXPORT ${PROJECT_NAME}
    LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
  )

  # This is local export file, which references files from local build tree.
  #export(EXPORT MiniROSTargets FILE "${CMAKE_CURRENT_BINARY_DIR}/cmake/${PROJECT_NAME}Targets.cmake")
  
  # This export file will be generated and installed to CMAKE_INSTALL_PREFIX
  install(EXPORT ${PROJECT_NAME}
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION lib/cmake/${PROJECT_NAME}
  )

  # Adding manual aliases, which match exported names.
  add_library(${PROJECT_NAME}::roscxx ALIAS roscxx)
  add_library(${PROJECT_NAME}::bag_storage ALIAS bag_storage)
endif(catkin_FOUND)


if (MINIROS_BUILD_TESTS)
	find_package(GTest 1.8.0)
	# Ensuring target "tests" is exists at a top level CMakeLists for compatibility with Eclipse CDT.
	if(TARGET tests)
	else ()
		add_custom_target(tests)
	endif()
		
	if (GTEST_FOUND)
		add_subdirectory(test)
	else()
		message(STATUS "Failed to find gtest. No tests will be built")
	endif()
endif()

if (MINIROS_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif ()

# uninstall target
if(NOT TARGET uninstall)
  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

  add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif()
