add_executable(test_duration rostime/duration.cpp)
target_link_libraries(test_duration roscxx)

add_executable(test_time rostime/time.cpp)
target_link_libraries(test_time roscxx)
