set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#include_directories(${MINIROS_INCLUDE_DIRS})

SET(rostime_SRC
	rostime/duration.cpp
	rostime/rate.cpp
	rostime/time.cpp)

set(cpp_common_SRC
	debug.cpp
	header.cpp
	console.cpp
	serialization.cpp)

set(rosbag_SRC "")
set(rosbag_LIBS "")
set(rosbag_INCLUDE "")

# Finding BZip2
if (MINIROS_USE_SYSTEM_BZIP2)
	find_package(BZip2 QUIET)
endif(MINIROS_USE_SYSTEM_BZIP2)

if (BZIP2_FOUND)
	add_definitions(${BZIP2_DEFINITIONS})
	list(APPEND rosbag_INCLUDE ${BZIP2_INCLUDE_DIR})
	list(APPEND rosbag_SRC rosbag_storage/bz2_stream.cpp)

	if (NOT MINIROS_USE_SYSTEM_BZIP)
		list(APPEND rosbag_LIBS bz2_import)
	else ()
		list(APPEND rosbag_LIBS ${BZIP2_LIBRARIES})
	endif ()
	
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
	list(APPEND rosbag_SRC
		rosbag_storage/lz4_stream.cpp
		roslz4/lz4s.c
		roslz4/xxhash.c)
	
	list(APPEND rosbag_LIBS ${lz4_LIBRARIES})
	list(APPEND rosbag_INCLUDE ${lz4_INCLUDE_DIRS})
	
	set_source_files_properties(src/lz4s.c src/xxhash.c
		PROPERTIES COMPILE_DEFINITIONS "XXH_NAMESPACE=ROSLZ4_")
	add_definitions(-DMINIBAG_HAS_LZ4)
	add_definitions()
endif()


set(AES_ENCRYPT_SOURCE "")
set(AES_ENCRYPT_LIBRARIES "")
if(NOT WIN32)
  set(AES_ENCRYPT_SOURCE "rosbag_storage/aes_encryptor.cpp" "rosbag_storage/gpgme_utils.cpp")
  set(AES_ENCRYPT_LIBRARIES "crypto" "gpgme")
endif()


if (MINIROS_BUILD_SHARED_LIBS)
  add_library(roscxx SHARED ${rostime_SRC} ${cpp_common_SRC})
else()
  add_library(roscxx ${rostime_SRC} ${cpp_common_SRC})
endif()

set_property(TARGET roscxx PROPERTY CXX_STANDARD 17)
target_link_libraries(roscxx ${console_bridge_LIBRARIES})
target_compile_definitions(roscxx PRIVATE rostime_EXPORTS miniros_EXPORTS)

set(bag_storage_SRC
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

if (MINIROS_BUILD_SHARED_LIBS)
	add_library(bag_storage SHARED ${bag_storage_SRC})
else ()
	add_library(bag_storage ${bag_storage_SRC})
endif()

target_include_directories(bag_storage PRIVATE ${MINIROS_INCLUDE_DIRS} ${rosbag_INCLUDE})
target_compile_definitions(bag_storage PRIVATE rosbag_EXPORTS rosbag_storage_EXPORTS roslz4_EXPORTS)

set_property(TARGET bag_storage PROPERTY CXX_STANDARD 17)
target_link_libraries(bag_storage ${AES_ENCRYPT_LIBRARIES} ${rosbag_LIBS} roscxx)

set(minibag_SRC
	rosbag/encrypt.cpp
	rosbag/play.cpp
	rosbag/player.cpp
	rosbag/record.cpp
	rosbag/recorder.cpp
	rosbag/time_translator.cpp)


set(MINIROS_LIBRARY roscxx PARENT_SCOPE)
set(MINIROS_EXPORT roscxx bag_storage ${rosbag_LIBS} PARENT_SCOPE)

if (catkin_FOUND)
	install(TARGETS ${MINIROS_EXPORT}
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
		EXPORT MiniROSTargets2
	)
endif()