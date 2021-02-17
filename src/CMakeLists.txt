if (NOT WIN32)
	#
else (NOT WIN32)
	#
endif (NOT WIN32)

include_directories(${MINIROS_INCLUDE_DIRS})
#include_directories(${EXTERNAL_INCLUDE})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(cpp_common debug.cpp header.cpp console.cpp)

SET(serialization_SRC serialization.cpp)
SET(rostime_SRC
	rostime/duration.cpp
	rostime/rate.cpp
	rostime/time.cpp)

set(rosbag_SRC "")
set(rosbag_LIBS "")

# Finding BZip2
if (MINIROS_USE_SYSTEM_BZIP2)
	find_package(BZip2 QUIET)
endif(MINIROS_USE_SYSTEM_BZIP2)

if(BZIP2_FOUND)
	add_definitions(${BZIP2_DEFINITIONS})
	include_directories(${BZIP2_INCLUDE_DIR})
	list(APPEND rosbag_SRC rosbag_storage/bz2_stream.cpp)
	list(APPEND rosbag_LIBS ${BZIP2_LIBRARIES})
	
	add_definitions(-DMINIBAG_HAS_BZIP2)
else(BZIP2_FOUND)
	message(STATUS "No bzip2 was found. Bzip2 support is disabled.")
endif(BZIP2_FOUND)

# Finding LZ4
if (MINIROS_USE_SYSTEM_LZ4)
	find_path(lz4_INCLUDE_DIRS NAMES lz4.h)
	find_library(lz4_LIBRARIES NAMES lz4 liblz4)
	
	if (NOT lz4_INCLUDE_DIRS OR NOT lz4_LIBRARIES)
		if (NOT lz4_INCLUDE_DIRS)
	    	message(WARNING "lz4 includes not found")
		endif()
		if (NOT lz4_LIBRARIES)
			message(WARNING "lz4 library not found")
		endif()		
	endif()
endif (MINIROS_USE_SYSTEM_LZ4)

if (lz4_LIBRARIES)
	list(APPEND rosbag_SRC rosbag_storage/lz4_stream.cpp)

	add_library(roslz4
		roslz4/lz4s.c
		roslz4/xxhash.c)
		
	target_link_libraries(roslz4 ${lz4_LIBRARIES})
	target_include_directories(roslz4 PRIVATE ${lz4_INCLUDE_DIRS})

	set_source_files_properties(src/lz4s.c src/xxhash.c
		PROPERTIES COMPILE_DEFINITIONS "XXH_NAMESPACE=ROSLZ4_")

	list(APPEND rosbag_LIBS roslz4)
	add_definitions(-DMINIBAG_HAS_LZ4)
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
target_link_libraries(minibag_storage ${AES_ENCRYPT_LIBRARIES} ${rosbag_LIBS} cpp_common)


add_library(miniroscpp ${serialization_SRC} ${rostime_SRC})
set_property(TARGET miniroscpp PROPERTY CXX_STANDARD 17)
target_link_libraries(miniroscpp ${console_bridge_LIBRARIES} minibag_storage cpp_common)


set(MINIROS_LIBRARY miniroscpp CACHE STRING "MiniROS C++ library name")
set(MINIROS_EXPORT miniroscpp minibag_storage cpp_common ${rosbag_LIBS} PARENT_SCOPE)

if (catkin_FOUND)
	install(TARGETS ${MINIROS_EXPORT}
		EXPORT MiniROSTargets
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
else()
	install(TARGETS ${MINIROS_EXPORT}
		EXPORT MiniROSTargets
		LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)
endif()