# Migration from ROS1 #

Most of the migration is done by renaming everything `ros::` to `miniros::`.

Original ROS1 code:

```c++
// From
#include <ros/ros.h>

ros::NodeHandle privateNh("~");
ros::Subscriber sub = privateNh.subscribe(...);

ros::Time now = ros::Time::now();
```

Miniros:

```c++
#include <miniros/ros.h>

miniros::NodeHandle privateNh("~");
miniros::Subscriber sub = privateNh.subscribe(...);

miniros::Time now = miniros::Time::now();
```

## CMake ##

Miniros does not rely on catkin or ament. It is just a plain CMake library.

```cmake
find_package(miniros 0.4.1 REQUIRED)

add_executable(my_ros_node main.cpp)

# All client code is exposed through miniros::roscxx library
target_link_libraries(my_ros_node miniros::roscxx)

# Miniros brings generated code for all standard messages, but they are not added by default to miniros client.
target_include_directories(my_ros_node PRIVATE ${MINIROS_GENERATED_INCLUDE_DIRS})


add_executable(some_rosbag_reader ...)
# 
target_link_libraries(some_rosbag_reader miniros::bag_storage)
```

# Custom messages and services #

There is no standalone generator for new messages. 