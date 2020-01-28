if (NOT WIN32)
	#
else (NOT WIN32)
	#
endif (NOT WIN32)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(cpp_common_SRC debug.cpp header.cpp console.cpp)
SET(serialization_SRC serialization.cpp)
SET(rostime_SRC
	rostime/duration.cpp
	rostime/rate.cpp
	rostime/time.cpp)

set(rosbag_SRC "")
set(rosbag_LIBS "")

# Finding BZip2
find_package(BZip2 QUIET)
if(BZIP2_FOUND)
	add_definitions(${BZIP2_DEFINITIONS})
	include_directories(${BZIP2_INCLUDE_DIR})
	list(APPEND rosbag_SRC rosbag_storage/bz2_stream.cpp)
	list(APPEND rosbag_LIBS ${BZIP2_LIBRARIES})
endif(BZIP2_FOUND)

# Finding LZ4
find_path(lz4_INCLUDE_DIRS NAMES lz4.h)
find_library(lz4_LIBRARIES NAMES lz4 liblz4)

if (NOT lz4_INCLUDE_DIRS OR NOT lz4_LIBRARIES)
	if (NOT lz4_INCLUDE_DIRS)
    	message(WARNING "lz4 includes not found")
	endif()
	if (NOT lz4_LIBRARIES)
		message(WARNING "lz4 library not found")
	endif()
else()
	include_directories(${lz4_INCLUDE_DIRS})
	list(APPEND rosbag_SRC rosbag_storage/lz4_stream.cpp)

	add_library(roslz4
		roslz4/lz4s.c
		roslz4/xxhash.c)

	set_source_files_properties(src/lz4s.c src/xxhash.c
		PROPERTIES COMPILE_DEFINITIONS "XXH_NAMESPACE=ROSLZ4_")

	list(APPEND rosbag_LIBS ${lz4_LIBRARIES} roslz4)	
endif()


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
	rosbag_storage/chunked_file.cpp
	rosbag_storage/message_instance.cpp
	rosbag_storage/query.cpp
	rosbag_storage/stream.cpp
	rosbag_storage/uncompressed_stream.cpp
	rosbag_storage/view.cpp
	
	${rosbag_SRC}
	rosbag_storage/no_encryptor.cpp
	${AES_ENCRYPT_SOURCE}
	)
#rosbag_storage/gpgme_utils.cpp

set(minibag_SRC
	rosbag/encrypt.cpp
	rosbag/play.cpp
	rosbag/player.cpp
	rosbag/record.cpp
	rosbag/recorder.cpp
	rosbag/time_translator.cpp)

set_property(TARGET minibag_storage PROPERTY CXX_STANDARD 17)
target_link_libraries(minibag_storage ${AES_ENCRYPT_LIBRARIES} ${rosbag_LIBS})

#target_link_libraries(minibag minibag_storage)

add_library(miniroscpp ${cpp_common_SRC} ${serialization_SRC} ${rostime_SRC})
set_property(TARGET miniroscpp PROPERTY CXX_STANDARD 17)
target_link_libraries(miniroscpp ${console_bridge_LIBRARIES} minibag_storage)

if (catkin_FOUND)
install(TARGETS miniroscpp minibag_storage
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
endif()