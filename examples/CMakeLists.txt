include_directories(${MINIROS_INCLUDE_GENERATED_DIRS})

add_executable(bag_read bag_read.cpp)
target_link_libraries(bag_read bag_storage roscxx)
set_target_properties (bag_read PROPERTIES FOLDER "Examples")


add_executable(bag_write bag_write.cpp)
target_link_libraries(bag_write bag_storage roscxx)
set_target_properties (bag_write PROPERTIES FOLDER "Examples")

# Example publisher and subscriber
# It follows official ROS tutorial at http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
add_executable(talker talker.cpp)
target_link_libraries(talker roscxx)
target_include_directories(talker PRIVATE ${MINIROS_INCLUDE_GENERATED_DIRS})

add_executable(listener listener.cpp)
target_link_libraries(listener roscxx)
target_include_directories(listener PRIVATE ${MINIROS_INCLUDE_GENERATED_DIRS})

# A benchmark for different types of logging.
add_executable(log_demo log_demo.cpp)
target_link_libraries(log_demo roscxx)
target_include_directories(log_demo PRIVATE ${MINIROS_INCLUDE_GENERATED_DIRS})
