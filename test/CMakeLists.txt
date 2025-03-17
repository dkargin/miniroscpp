include_directories(${MINIROS_INCLUDE_DIRS})

include(CTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

function(miniros_add_test TARGET_NAME)
	# These options are mapped to boolean flags
	set(bool_options WITH_MASTER DISABLED)
	set(single_value_options)
	cmake_parse_arguments(${CMAKE_CURRENT_FUNCTION} "${bool_options}" "${single_value_options}" "SOURCES;LIBRARIES" ${ARGN})
	add_executable(${TARGET_NAME} ${${CMAKE_CURRENT_FUNCTION}_SOURCES})
	set(optional_libs gtest_main)
	target_link_libraries(${TARGET_NAME} miniros::roscxx ${GTEST_LIBRARIES})

	if (NOT ${CMAKE_CURRENT_FUNCTION}_DISABLED)
		# Test is disabled. It is either not ported properly from rostest or does not suite CTest environment
		add_test(NAME ${TARGET_NAME} COMMAND ${TARGET_NAME})
		if (${CMAKE_CURRENT_FUNCTION}_MASTER)
			# Test needs master to operate
			# Fixtures are still not working
			# set_tests_properties(${TARGET_NAME} PROPERTIES FIXTURES_REQUIRED miniroscore)
		endif()
	endif()
	set_target_properties (${TARGET_NAME} PROPERTIES FOLDER "Tests")
endfunction()

miniros_add_test(test_duration SOURCES rostime/test_duration.cpp)
miniros_add_test(test_time SOURCES rostime/test_time.cpp)

add_subdirectory(xmlrpc)
add_subdirectory(roscpp/basic)
add_subdirectory(roscpp/advanced)
