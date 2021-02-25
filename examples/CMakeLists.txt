include_directories(${MINIROS_INCLUDE_GENERATED_DIRS})

add_executable(bag_read bag_read.cpp)
target_link_libraries(bag_read bag_storage roscxx)
set_target_properties (bag_read PROPERTIES FOLDER "Examples")


add_executable(bag_write bag_write.cpp)
target_link_libraries(bag_write bag_storage roscxx)
set_target_properties (bag_write PROPERTIES FOLDER "Examples")

