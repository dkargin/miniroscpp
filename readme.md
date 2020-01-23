# MiniROS cpp distribution #

MiniROS is a standalone ROS distribution with minimum external requirements.

# Plan #

1. Get rid of boost::format. It can be replaced by a local implementation.		DONE
1. Merge console_bridge
1. Figure out how to run new codegen with packages like sensor_msgs or nav_msgs.
1. Squash export macro headers, like miniros/macros.h, miniros/roscpp_serialization_macros.h, minibag/macros.h.
1. Adapt tests from corresponding libraries.
1. Merge rosbag in.
1. Check python code.
1. Adapt CMakeLists.txt to work without catkin.
1. Check if I can merge whole ROS transport in a library.

# References #

Current codebase is merged from multiple ROS packages:

roscpp_core/cpp_common

roscpp_core/roscpp_traits - all headers have gone to include/miniros/...

roscpp_core/roscpp_serialization - all code moved to include/miniros and src accordingly.

roscpp_core/rostime

ros_comm/utilities/roslz4 - used by rosbag

ros_comm/tools/rosbag_storage - core rosbag library. It was renamed to minibag and merged
