# MiniROS cpp distribution #

MiniROS is a standalone ROS distribution with minimum external requirements.

Contents:

1. All serialization code from roscpp.
1. rosbag_storage library for interaction with rosbag files from C++.
1. rostime. It is still not tested.
1. include/generated contains generated code for std_msgs and common_msg

# Building standalone library #

This project has reduced number of external dependencies and can be built using any c++14 compiler.

It also can work without catkin, but you still need catkin and [gencxx](https://github.com/dkargin/gencxx) to generate headers for new message types.

```
# We need to initialize git submodules to bzip2 and lz4
git submodule update --init
# Regular cmake build.
mkdir build
cd build
cmake ..
make
```

It generates `libminiroscpp.so` library, that contains all the code inside.

You can force miniros use system version of **bzip** by setting CMake option `MINIROS_USE_SYSTEM_BZIP2=ON`. `MINIROS_USE_SYSTEM_LZ4` does the same for **lz4**.

TODO: test install scripts

# Using miniros/rosbag as a submodule #

You can build miniros alongside with your project sources. The project can have the following structure:

```
some_project
  |- thirdparty
  |   \ miniroscpp
  | ...
  \- CMakeLists.txt
```

Somewhere in a root CMakeLists.txt:

```
# Adding miniros to the project.
add_subdirectory(thirdparty/miniroscpp)
```

Adding **miniros** to a target:

```
add_executable(some_executable main.cpp)

# Linking miniros using a direct name.
# All necessary includes will be added to target `some_executable` automatically
target_link_libraries(some_executable miniros::roscxx miniros::bag_storage)

# Adding paths to pregenerated headers.
target_include_directories(some_executable PRIVATE ${MINIROS_INCLUDE_GENERATED_DIRS})
```

# Current status #

1. ROSBag C++ client is complete. It is an independent library. No boost or big external libraries are required.
1. Ported ROS C++ client code. It still uses boost::signals2 and boost::thread_unique_ptr internally, but they are not leaked to user code (hopefully).
1. Ported rosbag play and record utilities.
1. Added some tests from original ros_comm package. Though they do not work without some external rosmaster.

Missing things:

1. This is not a complete ROS distribution. You still need regular ROS tools, like roslaunch, rostopic or rosmaster somewhere else. This is just a portable C++ client, which helps connecting ROS system from the projects, where using regular ROS is not the best idea.
1. ROS logging is probably broken right now.
1. Full testing from ros_comm is also missing.

# Future plans #

Since **miniros** is experimental distribution, I am free to improve core API. These are possible directions:

1. Replace boost::signal2 and boost::thread_unique_ptr by a local version.
1. Rework global initialization: ros::init should return some sort of a context.
  All global variables should be moved to this context.
  `NodeHandle` should use either this context, or parent node in its constructor.
  This alows implementing nodelet approach without using separate API.
1. Ticket-based API for ros services: resolve issues with crashing server.
1. Reduce compilation time by moving some exposed class fields to "Impl" section.
1. Test everything on android.
1. Test everything on windows.

# ROS References #

Current codebase is an adaptation of the following ROS packages:

** roscpp_core ** (http://wiki.ros.org/roscpp_core)

 - cpp_common
 - roscpp_traits - all headers have gone to include/miniros/...
 - roscpp_serialization - all code moved to include/miniros and src accordingly.
 - rostime - all code moved to include/miniros and src accordingly.

** ros_comm** (http://wiki.ros.org/ros_comm)

 - roslz4 - used by rosbag. Moved to src/roslz4.
 - tools/rosbag_storage - core rosbag library. It was renamed to minibag and merged.
 
** console_bridge** (http://wiki.ros.org/console_bridge)

** topic_tools**
  - ShapeShifter - it was ported here for rosbag app.

# Additional packages used #

## Bzip2 ##

I use experimental version of bzip2 from https://gitlab.com/federicomenaquintero/bzip2. It has handy CMake support.

## LZ4 ##

I am using official [lz4](https://github.com/lz4/lz4.git)

# License #

This code is licenced under BSD-3 license.
Original code was licenced under BSD-3 license as well.
