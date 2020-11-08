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
target_link_libraries(some_executable miniroscpp)

# Linking miniros using exported variable.
target_link_libraries(some_executable ${MINIROS_LIBRARY})

# Adding paths to pregenerated headers.
target_include_directories(some_executable PRIVATE ${MINIROS_INCLUDE_GENERATED_DIRS})
```

# Plan #

1. ~~Get rid of boost::format. It can be replaced by a local implementation.~~ DONE
1. ~~Merge console_bridge.~~ DONE
1. ~~Figure out how to run new codegen with packages like sensor_msgs or nav_msgs.~~ DONE
1. ~~Squash export macro headers, like miniros/macros.h, miniros/roscpp_serialization_macros.h, minibag/macros.h.~~ DONE
1. Adapt tests from corresponding libraries.
1. ~~Merge rosbag_storage inside.~~ DONE
1. ~~Add local versions of **bzip2** and **lz4**.~~ DONE
1. Check python code.
1. ~~Adapt CMakeLists.txt to work without catkin.~~ DONE
1. Check if I can merge whole ROS transport in a library.
1. Provide a proper install and configuration scripts for CMake.
1. Test library on android.

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

# Additional packages used #

## Bzip2 ##

I use experimental version of bzip2 from https://gitlab.com/federicomenaquintero/bzip2. It has handy CMake support.

## LZ4 ##

I am using official [lz4](https://github.com/lz4/lz4.git)

# License #

This code is licenced under BSD-3 license.
Original code was licenced under BSD-3 license as well.
 