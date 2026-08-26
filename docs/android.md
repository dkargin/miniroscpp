# Building miniros for Android (CMake + NDK)

miniros is a native C++ library. On Android it is built with **CMake and the NDK only** — no Gradle / Android Gradle Plugin.

Downstream apps should consume miniros via CMake (`add_subdirectory`, or link the installed static library and headers).

## Requirements

- CMake 3.22+ (host)
- Ninja (recommended) or Unix Makefiles
- Android NDK **r22 or newer**
  - r22 is the minimum because miniros uses `std::filesystem`, which landed in libc++ in NDK r22
  - Newer NDKs (e.g. r28) are fine
- `adb` from the Android SDK platform-tools (for device smoke tests)

## Configure and build

Use a separate build directory (do not reuse a host Linux `build/`).

```bash
export ANDROID_NDK_HOME=/path/to/ndk/22.1.7171670   # or any r22+
# Example on this machine:
# export ANDROID_NDK_HOME=/media/dkargin/dkargin-bags/Android/SDK/ndk/28.2.13676358

cmake -S . -B build-android -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE="$ANDROID_NDK_HOME/build/cmake/android.toolchain.cmake" \
  -DANDROID_ABI=arm64-v8a \
  -DANDROID_PLATFORM=android-28 \
  -DANDROID_STL=c++_static \
  -DCMAKE_BUILD_TYPE=Release

cmake --build build-android --target miniroscore talker listener -j"$(nproc)"
```

Binaries land in `build-android/bin/`.

Match `ANDROID_ABI` to the device (`adb shell getprop ro.product.cpu.abi`). Use `x86_64` for many emulators.

## What `if(ANDROID)` does in the root CMakeLists

When the NDK toolchain sets `ANDROID`, the project automatically:

- Builds **static** `roscxx` (`MINIROS_BUILD_SHARED=OFF`)
- Enables examples; disables tests and host system deps
- Links `miniroscore`, `talker`, and `listener` against the static library
- Uses `c++_static` from the command line above so tools need no extra `.so` on device

## ADB smoke test

```bash
adb push build-android/bin/miniroscore /data/local/tmp/
adb push build-android/bin/talker /data/local/tmp/
adb push build-android/bin/listener /data/local/tmp/
adb shell chmod 755 /data/local/tmp/miniroscore /data/local/tmp/talker /data/local/tmp/listener

# terminal 1
adb shell 'cd /data/local/tmp && ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1 ./miniroscore'

# terminal 2
adb shell 'cd /data/local/tmp && ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1 ./listener'

# terminal 3
adb shell 'cd /data/local/tmp && ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1 ./talker'
```

Set `ROS_IP` (or `ROS_HOSTNAME`): on Android, some host-interface discovery paths are skipped.

## Using miniros from another Android CMake project

```cmake
set(ANDROID_STL c++_static)  # or as toolchain/Gradle args in the app
add_subdirectory(path/to/miniroscpp miniros)

add_library(my_native_module SHARED my_code.cpp)
target_link_libraries(my_native_module PRIVATE roscxx)  # static archive when ANDROID
```

Or install into a prefix and use `find_package` / imported targets from that prefix.
