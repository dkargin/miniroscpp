# MiniROS cpp distribution #

MiniROS is a standalone ROS distribution with minimum external requirements.

Supported platforms:

 - [![Linux](https://github.com/dkargin/miniroscpp/actions/workflows/cmake-multiplatform.yml/badge.svg)](https://github.com/dkargin/miniroscpp/actions/workflows/cmake-multiplatform.yml)
 - [![Windows](https://github.com/dkargin/miniroscpp/actions/workflows/cmake-win32.yml/badge.svg)](https://github.com/dkargin/miniroscpp/actions/workflows/cmake-win32.yml)

Contents:

1. All serialization and transport code from roscpp.
2. rosbag_storage library for interaction with rosbag files from C++.
3. rostime. It is still not tested.
4. include/generated contains generated code for std_msgs and common_msg

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

It generates `libminiroscpp.so` library. It contains all client code inside. Rosbag client API is located in `libbag_storage.so`.

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

More detailed migration guide can be found at [Migration guide](docs/migration.md)

# Current status #

1. ROSBag C++ client is complete. It is an independent library. No boost or big external libraries are required.
2. Ported ROS C++ client code to use only c++17.
3. Ported rosbag play and record utilities. Now they invoked through `minibag record` and `minibag play` commands.
4. Added some tests from original ros_comm package. Though they do not work without some external rosmaster.
5. **miniroscore** is pure C++ rewrite of **roscore**. **miniros::Master** can be embedded into user application.

Missing things:

1. ROS logging is probably broken right now. MINIROS_INFO works somehow, but other log levels are probably broken. 
2. Testing suite from ros_comm not completely backported. I need to replace rostest invocations with CMake/CTest scripts.
3. No additional utilities like roslaunch, rostopic, rosnode, ... .
4. No complicated things like actionlib, bond, nodelet, ... . 
5. `minibag info` is not implemented.

# Future plans #

Since **miniros** is experimental distribution, I am free to improve core API. These are possible directions:

1. Rework global initialization: ros::init should return some sort of context.
  All global variables should be moved to this context.
  `NodeHandle` should use either this context, or parent node in its constructor.
  This allows implementing nodelet approach without using separate API.
2. Ticket-based API for ros services: resolve issues with crashing server.
3. Reduce compilation time by moving some exposed class fields to "Impl" section.
4. Test everything on android.
5. Test everything on windows.

# ROS References #

Current codebase is an adaptation of the following ROS packages:

**roscpp_core** (http://wiki.ros.org/roscpp_core)

 - cpp_common
 - roscpp_traits - all headers have gone to include/miniros/...
 - roscpp_serialization - all code moved to include/miniros and src accordingly.
 - rostime - all code moved to include/miniros and src accordingly.

**ros_comm** (http://wiki.ros.org/ros_comm)

 - roslz4 - used by rosbag. Moved to src/roslz4.
 - tools/rosbag_storage - core rosbag library. It was renamed to minibag and merged.
 
**console_bridge** (http://wiki.ros.org/console_bridge)

**topic_tools**
  - ShapeShifter - it was ported here for rosbag app.

# Additional packages used #

## Bzip2 ##

Miniros uses experimental version of bzip2 from https://gitlab.com/federicomenaquintero/bzip2. It has handy CMake support.

## LZ4 ##

Miniros uses official [lz4](https://github.com/lz4/lz4.git)

# License #

Most of miniros code is licenced under BSD-3 license, the same as original roscpp code.
XmlRpc is included into codebase and has LGPL2 licence.
