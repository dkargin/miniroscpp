if (NOT WIN32)
	#
else (NOT WIN32)
	#
endif (NOT WIN32)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(cpp_common_SRC debug.cpp header.cpp)
SET(serialization_SRC serialization.cpp)
SET(rostime_SRC
	rostime/duration.cpp
	rostime/rate.cpp
	rostime/time.cpp)
	
add_library(roslz4
	roslz4/lz4s.c
	roslz4/xxhash.c)

set_source_files_properties(
  src/lz4s.c src/xxhash.c
PROPERTIES COMPILE_DEFINITIONS "XXH_NAMESPACE=ROSLZ4_")

find_package(BZip2 REQUIRED)

add_definitions(${BZIP2_DEFINITIONS})

find_path(lz4_INCLUDE_DIRS NAMES lz4.h)
if (NOT lz4_INCLUDE_DIRS)
  message(FATAL_ERROR "lz4 includes not found")
endif()

find_library(lz4_LIBRARIES NAMES lz4 liblz4)
if (NOT lz4_LIBRARIES)
  message(FATAL_ERROR "lz4 library not found")
endif()
set(lz4_FOUND TRUE)
include_directories(${BZIP2_INCLUDE_DIR})
include_directories(${lz4_INCLUDE_DIRS})

set(AES_ENCRYPT_SOURCE "")
set(AES_ENCRYPT_LIBRARIES "")
if(NOT WIN32)
  set(AES_ENCRYPT_SOURCE "rosbag_storage/aes_encryptor.cpp" "rosbag_storage/gpgme_utils.cpp")
  set(AES_ENCRYPT_LIBRARIES "crypto" "gpgme")
endif()

add_library(minibag_storage
	rosbag_storage/bag_player.cpp
	rosbag_storage/bag.cpp
	rosbag_storage/buffer.cpp
	rosbag_storage/bz2_stream.cpp
	rosbag_storage/chunked_file.cpp
	rosbag_storage/lz4_stream.cpp
	rosbag_storage/message_instance.cpp
	rosbag_storage/query.cpp
	rosbag_storage/stream.cpp
	rosbag_storage/uncompressed_stream.cpp
	rosbag_storage/view.cpp
	
	rosbag_storage/no_encryptor.cpp
	${AES_ENCRYPT_SOURCE}
	)
#rosbag_storage/gpgme_utils.cpp

set_property(TARGET minibag_storage PROPERTY CXX_STANDARD 17)
target_link_libraries(minibag_storage roslz4 ${BZIP2_LIBRARIES} ${lz4_LIBRARIES} ${AES_ENCRYPT_LIBRARIES})

add_library(miniroscpp ${cpp_common_SRC} ${serialization_SRC} ${rostime_SRC})
set_property(TARGET miniroscpp PROPERTY CXX_STANDARD 17)
target_link_libraries(miniroscpp ${console_bridge_LIBRARIES} minibag_storage)

install(TARGETS miniroscpp minibag_storage
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})