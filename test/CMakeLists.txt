include_directories(${MINIROS_INCLUDE_DIRS})

add_executable(test_duration rostime/test_duration.cpp)
target_link_libraries(test_duration roscxx ${GTEST_LIBRARIES})

add_executable(test_time rostime/test_time.cpp)
target_link_libraries(test_time roscxx ${GTEST_LIBRARIES})

add_subdirectory(xmlrpc)