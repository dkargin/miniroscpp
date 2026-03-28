include_directories(${MINIROS_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

include(CTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(manage-master manage-master.cpp)
target_link_libraries(manage-master miniros::roscxx)
add_dependencies(manage-master miniroscore)

set(TESTS_WITH_MASTER "" CACHE INTERNAL "Tests with rosmaster")

set(TEST_LAUNCHER ${CMAKE_CURRENT_SOURCE_DIR}/launch_test.sh)

function(miniros_add_test TARGET_NAME)
	# These options are mapped to boolean flags
	set(bool_options WITH_MASTER DISABLED)
	set(single_value_options)
	cmake_parse_arguments(${CMAKE_CURRENT_FUNCTION} "${bool_options}" "${single_value_options}" "SOURCES;LIBRARIES" ${ARGN})
	add_executable(${TARGET_NAME} ${${CMAKE_CURRENT_FUNCTION}_SOURCES})
	set(optional_libs gtest_main)
	target_link_libraries(${TARGET_NAME} roscxx ${GTEST_LIBRARIES})

	if (${CMAKE_CURRENT_FUNCTION}_DISABLED)
		# Test is disabled. It is either not ported properly from rostest or does not suite CTest environment
	else()
		# Running test through helper script.
		# It helps catching call stacks on remote CI machine.
		add_test(NAME ${TARGET_NAME} COMMAND ${TEST_LAUNCHER} $<TARGET_FILE:${TARGET_NAME}>)
		if (${CMAKE_CURRENT_FUNCTION}_WITH_MASTER)
			# Test needs master to operate
			# Fixtures are still not working
			set_tests_properties(start-master PROPERTIES FIXTURES_SETUP ${TARGET_NAME})
			set_tests_properties(stop-master PROPERTIES FIXTURES_CLEANUP ${TARGET_NAME})
			message(STATUS "Test ${TARGET_NAME} will require rosmaster")
		endif()
	endif()
	set_target_properties (${TARGET_NAME} PROPERTIES FOLDER "Tests")
endfunction()

miniros_add_test(test_duration SOURCES rostime/test_duration.cpp)
miniros_add_test(test_time SOURCES rostime/test_time.cpp)

add_subdirectory(xmlrpc)
add_subdirectory(roscpp/basic)
add_subdirectory(roscpp/advanced)
