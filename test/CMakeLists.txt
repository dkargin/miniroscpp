include_directories(${MINIROS_INCLUDE_DIRS})

function(miniros_add_test TARGET_NAME SOURCES)
	set (EXTRA_ARGS ${ARGN})
	list(LENGTH EXTRA_ARGS NUM_EXTRA_ARGS)

	if (${NUM_EXTRA_ARGS} GREATER 0)
		list(GET EXTRA_ARGS 0 OPTIONAL_ARG)
		add_executable(${TARGET_NAME} ${SOURCES} ${OPTIONAL_ARG})
	else()
		add_executable(${TARGET_NAME} ${SOURCES})
	endif()
	target_link_libraries(${TARGET_NAME} roscxx ${GTEST_LIBRARIES})
	add_test(NAME ${TARGET_NAME} COMMAND ${TARGET_NAME})
	set_target_properties (${TARGET_NAME} PROPERTIES FOLDER "Tests")
endfunction()

miniros_add_test(test_duration rostime/test_duration.cpp)
miniros_add_test(test_time rostime/test_time.cpp)

add_subdirectory(xmlrpc)

add_subdirectory(roscpp/basic)
