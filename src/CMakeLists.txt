set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(replacements)

# Injecting flags for debug.

# execinfo.h is needed for backtrace on glibc systems
CHECK_INCLUDE_FILE(execinfo.h HAVE_EXECINFO_H)
# do we have demangle capability?
# CHECK_INCLUDE_FILE doesn't work here for some reason
CHECK_CXX_SOURCE_COMPILES("#include<cxxabi.h>\nint main(int argc,char**argv){}" HAVE_CXXABI_H)
CHECK_FUNCTION_EXISTS(backtrace HAVE_GLIBC_BACKTRACE)

# Not everybody has <ifaddrs.h> (e.g., embedded arm-linux)
CHECK_INCLUDE_FILES(ifaddrs.h HAVE_IFADDRS_H)
# Not everybody has trunc (e.g., Windows, embedded arm-linux)
CHECK_FUNCTION_EXISTS(trunc HAVE_TRUNC)
# Not everybody has epoll (e.g., Windows, BSD, embedded arm-linux) 
CHECK_CXX_SYMBOL_EXISTS(epoll_wait "sys/epoll.h" HAVE_EPOLL)

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/internal_config.h.in" internal_config.h)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

SET(rostime_SRC
    rostime/duration.cpp
    rostime/rate.cpp
    rostime/time.cpp)

set(cpp_common_SRC
    debug.cpp
    header.cpp
    console.cpp
    console_print.cpp
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
endif()

set(ENCRYPT_SOURCE "")
set(ENCRYPT_LIBRARIES "")

# AES encryption plugin
if (MINIROS_ROSBAG_USE_OPENSSL_ENCRYPTION)
	find_package(OpenSSL QUIET)
	if (OpenSSL_FOUND)
		message(STATUS "Found OpenSSL include=${OPENSSL_INCLUDE_DIR}")
		list(APPEND ENCRYPT_SOURCE "rosbag_storage/aes_encryptor.cpp")
		list(APPEND ENCRYPT_LIBRARIES OpenSSL::Crypto)
	else ()
		message(STATUS "No OpenSSL was found. Rosbag will not support AES encryption.")
	endif()
endif()


if(MINIROS_ROSBAG_USE_GPGME_ENCRYPTION)
	list(APPEND ENCRYPT_SOURCE "rosbag_storage/gpgme_utils.cpp")
	list(APPEND ENCRYPT_LIBRARIES gpgme)
endif()


set(transport_SRC
  transport/master.cpp
  transport/network.cpp
  transport/subscriber.cpp
  transport/common.cpp
  transport/publisher_link.cpp
  transport/service_publication.cpp
  transport/connection.cpp
  transport/single_subscriber_publisher.cpp
  transport/param.cpp
  transport/service_server.cpp
  transport/wall_timer.cpp
  transport/xmlrpc_manager.cpp
  transport/publisher.cpp
  transport/timer.cpp
  transport/io.cpp
  transport/names.cpp
  transport/topic.cpp
  transport/topic_manager.cpp
  transport/poll_manager.cpp
  transport/publication.cpp
  transport/statistics.cpp
  transport/intraprocess_subscriber_link.cpp
  transport/intraprocess_publisher_link.cpp
  transport/callback_queue.cpp
  transport/service_server_link.cpp
  transport/service_client.cpp
  transport/node_handle.cpp
  transport/connection_manager.cpp
  transport/file_log.cpp
  transport/transport/transport.cpp
  transport/transport/transport_udp.cpp
  transport/transport/transport_tcp.cpp
  transport/subscriber_link.cpp
  transport/service_client_link.cpp
  transport/transport_publisher_link.cpp
  transport/transport_subscriber_link.cpp
  transport/service_manager.cpp
  transport/rosout_appender.cpp
  transport/init.cpp
  transport/subscription.cpp
  transport/subscription_queue.cpp
  transport/spinner.cpp
  transport/internal_timer_manager.cpp
  transport/message_deserializer.cpp
  transport/poll_set.cpp
  transport/service.cpp
  transport/this_node.cpp
  transport/steady_timer.cpp
  shape_shifter.cpp
  )

set(BOOST_COMPONENTS system thread)
if (MINIROS_USE_SYSTEM_PROGRAM_OPTIONS)
    list(APPEND BOOST_COMPONENTS program_options)
endif()

find_package(Boost REQUIRED COMPONENTS ${BOOST_COMPONENTS})
if (MINIROS_USE_SYSTEM_PROGRAM_OPTIONS)
    add_library(impl::program_options ALIAS Boost::program_options)
endif ()

if (MINIROS_BUILD_SHARED_LIBS)
  add_library(roscxx SHARED ${rostime_SRC} ${cpp_common_SRC} ${transport_SRC})
else()
  add_library(roscxx ${rostime_SRC} ${cpp_common_SRC} ${transport_SRC})
endif()

set_property(TARGET roscxx PROPERTY CXX_STANDARD 17)

target_link_libraries(roscxx PRIVATE xmlrpcpp rt stdc++fs ${console_bridge_LIBRARIES} ${Boost_LIBRARIES})

target_compile_definitions(roscxx PRIVATE rostime_EXPORTS miniros_EXPORTS)

target_include_directories(roscxx PRIVATE
    ${MINIROS_INCLUDE_DIRS}
    ${MINIROS_INCLUDE_TRANSPORT}
    ${MINIROS_INCLUDE_ROOT}
    ${MINIROS_INCLUDE_GENERATED_DIRS}
)

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
    ${ENCRYPT_SOURCE}
)

if (MINIROS_BUILD_SHARED_LIBS)
	add_library(bag_storage SHARED ${bag_storage_SRC})
else ()
	add_library(bag_storage ${bag_storage_SRC})
endif()

target_include_directories(bag_storage PRIVATE ${MINIROS_INCLUDE_DIRS} ${rosbag_INCLUDE})
target_compile_definitions(bag_storage PRIVATE rosbag_EXPORTS rosbag_storage_EXPORTS roslz4_EXPORTS)

set_property(TARGET bag_storage PROPERTY CXX_STANDARD 17)
target_link_libraries(bag_storage ${ENCRYPT_LIBRARIES} ${rosbag_LIBS} roscxx)


if (MINIROS_BUILD_ROSBAG_APPS)
    # Common library for rosbag applications.
    add_library(bag_app
        rosbag/player.cpp
        rosbag/recorder.cpp
        rosbag/time_translator.cpp
    )
    target_link_libraries(bag_app bag_storage roscxx)
    target_include_directories(bag_app PRIVATE ${MINIROS_INCLUDE_DIRS} ${MINIROS_INCLUDE_GENERATED_DIRS})

    # ROSBag recorder app
    add_executable(record rosbag/record.cpp)
    target_link_libraries(record bag_app impl::program_options)
    target_include_directories(record PRIVATE ${MINIROS_INCLUDE_DIRS} ${MINIROS_INCLUDE_GENERATED_DIRS})

    # ROSBag player app.
    add_executable(play rosbag/play.cpp)
    target_link_libraries(play bag_app impl::program_options)
    target_include_directories(play PRIVATE ${MINIROS_INCLUDE_DIRS} ${MINIROS_INCLUDE_GENERATED_DIRS})
endif()

set(MINIROS_LIBRARY roscxx PARENT_SCOPE)
set(MINIROS_EXPORT roscxx bag_storage ${rosbag_LIBS} PARENT_SCOPE)
