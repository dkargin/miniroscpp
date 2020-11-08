
add_executable(bag_read bag_read.cpp)
target_link_libraries(bag_read minibag_storage)

add_executable(bag_write bag_write.cpp)
target_link_libraries(bag_write minibag_storage)
