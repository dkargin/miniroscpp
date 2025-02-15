# Migration from ROS1 #

Most of the migration is done by renaming everything `ros::` to `miniros::`.

Original ROS1 code:

```c++
#include <ros/ros.h>

ros::NodeHandle privateNh("~");
ros::Subscriber sub = privateNh.subscribe(...);

ros::Time now = ros::Time::now();
ROS_INFO("System has started");
```

Miniros:

```c++
#include <miniros/ros.h>

miniros::NodeHandle privateNh("~");
miniros::Subscriber sub = privateNh.subscribe(...);

miniros::Time now = miniros::Time::now();
MINIROS_INFO("System has started");
```

## CMake ##

Miniros does not rely on catkin or ament. It is just a plain CMake library.

```cmake
find_package(miniros REQUIRED)

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
You will need some separate catkin workspace and gencxx package to generate code for new messages.

# Differences and incompatibilities #

1. Global functions from **ros::param::** and **ros::master::** are replaced by **miniros::MasterLink** class.
It contains all the methods of both param and master, but allows to simplify internal state.

2. Only a subset of logging macros is supported.
3. No plans for supporting actionlib for now.
4. miniros::init will be slightly changed to allow applications continue working without proper connection to master. 
5. ...