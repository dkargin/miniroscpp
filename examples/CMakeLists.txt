include_directories(${MINIROS_INCLUDE_GENERATED_DIRS})

add_executable(bag_read bag_read.cpp)
target_link_libraries(bag_read miniroscpp)

add_executable(bag_write bag_write.cpp)
target_link_libraries(bag_write miniroscpp)
