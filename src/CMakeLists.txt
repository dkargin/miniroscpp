add_subdirectory(replacements)


if (PkgConfig_FOUND)
  pkg_check_modules(PKG_LIBS REQUIRED libsystemd)
  if (PKG_LIBS_FOUND)
    set(LIBSYSTEMD_FOUND TRUE)
  else()
    set(LIBSYSTEMD_FOUND FALSE)
  endif()
else ()
  set(LIBSYSTEMD_FOUND FALSE)
endif()

message(STATUS "Building miniros for platform ${CMAKE_SYSTEM_NAME}")

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
  if (LIBSYSTEMD_FOUND AND MINIROS_USE_LIBSYSTEMD)
    message(STATUS "Will enable systemd support")
    set(HAVE_LIBSYSTEMD TRUE)
  endif()
endif()

# execinfo.h is needed for backtrace on glibc systems
CHECK_INCLUDE_FILE(execinfo.h HAVE_EXECINFO_H)
# do we have demangle capability?
# CHECK_INCLUDE_FILE doesn't work here for some reason
CHECK_CXX_SOURCE_COMPILES("#include<cxxabi.h>\nint main(int argc,char**argv){}" HAVE_CXXABI_H)
CHECK_FUNCTION_EXISTS(backtrace HAVE_GLIBC_BACKTRACE)

# Not everybody has <ifaddrs.h> (e.g., embedded arm-linux)
CHECK_INCLUDE_FILES(ifaddrs.h HAVE_IFADDRS_H)
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
    errors.cpp
    header.cpp
    console.cpp
    common.cpp
    console_print.cpp
    file_log.cpp
    launcher.cpp
    names.cpp
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

    if (NOT MINIROS_USE_SYSTEM_BZIP2)
            list(APPEND rosbag_INTERNAL_LIBS bz2_import)
    else ()
            list(APPEND rosbag_EXTERNAL_LIBS ${BZIP2_LIBRARIES})
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
	
  if (MINIROS_USE_SYSTEM_LZ4)
	  list(APPEND rosbag_EXTERNAL_LIBS ${lz4_LIBRARIES})
  else ()
    list(APPEND rosbag_INTERNAL_LIBS ${lz4_LIBRARIES})
  endif ()
	list(APPEND rosbag_INCLUDE ${lz4_INCLUDE_DIRS})
	
	set_source_files_properties(src/lz4s.c src/xxhash.c
		PROPERTIES COMPILE_DEFINITIONS "XXH_NAMESPACE=ROSLZ4_")
        add_definitions(-DMINIBAG_HAS_LZ4)
endif()

set(ENCRYPT_SOURCE "")
set(ENCRYPT_LIBRARIES "")

if(MINIROS_ROSBAG_USE_GPGME_ENCRYPTION)
  find_package(Gpgme REQUIRED)

  list(APPEND ENCRYPT_SOURCE "rosbag_storage/gpgme_utils.cpp")
  list(APPEND rosbag_INCLUDE ${GPGME_INCLUDES})
  list(APPEND ENCRYPT_LIBRARIES gpgme)
endif()

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

set(XMLRPC_SRC
    xmlrpc/XmlRpcClient.cpp
    xmlrpc/XmlRpcDispatch.cpp
    xmlrpc/XmlRpcServer.cpp
    xmlrpc/XmlRpcServerConnection.cpp
    xmlrpc/XmlRpcServerMethod.cpp
    xmlrpc/XmlRpcSocket.cpp
    xmlrpc/XmlRpcSource.cpp
    xmlrpc/XmlRpcUtil.cpp
    xmlrpc/XmlRpcValue.cpp
    xmlrpc/b64decode.cpp
    xmlrpc/b64encode.cpp
)

set(transport_SRC
    shape_shifter.cpp
    observer.cpp

    http/endpoints/xmlrpc.cpp
    http/endpoints/filesystem.cpp
    http/http_filters.cpp
    http/http_server.cpp
    http/http_server_connection.cpp
    http/http_tools.cpp

    network/network.cpp
    network/net_adapter.cpp
    network/net_address.cpp
    network/socket.cpp
    network/url.cpp

    transport/advertise_service_options.cpp
    transport/callback_queue.cpp
    transport/connection.cpp
    transport/connection_manager.cpp
    transport/init.cpp
    transport/intraprocess_subscriber_link.cpp
    transport/intraprocess_publisher_link.cpp
    transport/internal_timer_manager.cpp
    transport/io.cpp

    transport/master_link.cpp
    transport/message_deserializer.cpp

    transport/node_handle.cpp
    transport/poll_manager.cpp
    transport/poll_set.cpp
    transport/publication.cpp
    transport/publisher.cpp
    transport/publisher_link.cpp
    transport/rpc_manager.cpp
    transport/rosout_appender.cpp

    transport/service.cpp

    transport/service_client.cpp
    transport/service_client_link.cpp

    transport/service_publication.cpp
    transport/service_manager.cpp
    transport/service_server.cpp
    transport/service_server_link.cpp

    transport/single_subscriber_publisher.cpp
    transport/spinner.cpp
    transport/statistics.cpp
    transport/steady_timer.cpp

    transport/subscriber.cpp
    transport/subscriber_link.cpp
    transport/subscription.cpp
    transport/subscription_queue.cpp

    transport/timer.cpp
    transport/this_node.cpp
    transport/topic.cpp
    transport/topic_manager.cpp
    transport/transport/transport.cpp
    transport/transport/transport_udp.cpp
    transport/transport/transport_tcp.cpp
    transport/transport_publisher_link.cpp
    transport/transport_subscriber_link.cpp
    transport/wall_timer.cpp
    tf2/transform_broadcaster.cpp
  )

# minimaster contains core rosmaster API. It can be embedded to some other apps.
set(master_SRC
    roscore/discovery.cpp
    roscore/master.cpp
    roscore/master_endpoints.cpp
    roscore/master_handler.cpp
    roscore/master_html_printers.cpp
    roscore/node_ref.cpp
    roscore/registrations.cpp
    roscore/registration_manager.cpp
    roscore/resolver.cpp
    roscore/requester_info.cpp
    roscore/parameter_storage.cpp
    roscore/rosout.cpp
)

if (MINIROS_USE_SYSTEM_PROGRAM_OPTIONS)
    find_package(Boost REQUIRED COMPONENTS program_options)
    add_library(impl::program_options ALIAS Boost::program_options)
endif ()

if (MINIROS_BUILD_SHARED_LIBS)
  add_library(roscxx SHARED ${cpp_common_SRC} ${transport_SRC} ${master_SRC} ${rostime_SRC} ${XMLRPC_SRC})
else()
  add_library(roscxx ${cpp_common_SRC} ${transport_SRC} ${master_SRC} ${rostime_SRC} ${XMLRPC_SRC})
endif()

set_property(TARGET roscxx PROPERTY CXX_STANDARD 17)

if(WIN32)
  set(PLATFORM_LIBS wsock32 ws2_32)
else()
  set(PLATFORM_LIBS stdc++fs)
endif()

target_link_libraries(roscxx
  PRIVATE
    ${console_bridge_LIBRARIES} ${PKG_LIBS_LIBRARIES} ${PLATFORM_LIBS}
  PUBLIC
    ${CMAKE_THREAD_LIBS_INIT}
)

target_compile_definitions(roscxx PRIVATE rostime_EXPORTS miniros_EXPORTS)

target_include_directories(roscxx
PRIVATE
    ${MINIROS_INCLUDE_ROOT}/miniros
PUBLIC
    $<BUILD_INTERFACE:${MINIROS_INCLUDE_DIRS}>
    $<BUILD_INTERFACE:${MINIROS_INCLUDE_ROOT}>
    $<BUILD_INTERFACE:${MINIROS_INCLUDE_GENERATED_DIRS}>
    $<INSTALL_INTERFACE:include/generated>  # <prefix>/include/mylib
)

find_package(PkgConfig QUIET)

add_executable(miniroscore roscore/miniroscore.cpp)

target_link_libraries(miniroscore roscxx impl::program_options)
target_include_directories(miniroscore PRIVATE ${MINIROS_INCLUDE_ROOT} ${MINIROS_INCLUDE_GENERATED_DIRS})

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

target_include_directories(bag_storage PRIVATE ${MINIROS_INCLUDE_ROOT} ${rosbag_INCLUDE})
target_compile_definitions(bag_storage PRIVATE rosbag_EXPORTS rosbag_storage_EXPORTS roslz4_EXPORTS)

set_property(TARGET bag_storage PROPERTY CXX_STANDARD 17)
target_link_libraries(bag_storage PRIVATE ${ENCRYPT_LIBRARIES} ${rosbag_INTERNAL_LIBS} ${rosbag_EXTERNAL_LIBS} roscxx)

add_executable(minibag
    rosbag/main.cpp
    rosbag/player.cpp
    rosbag/recorder.cpp
    rosbag/time_translator.cpp
)
target_link_libraries(minibag bag_storage roscxx impl::program_options)
target_include_directories(minibag PRIVATE ${MINIROS_INCLUDE_DIRS} ${MINIROS_INCLUDE_GENERATED_DIRS})

set(MINIROS_LIBRARY roscxx PARENT_SCOPE)
set(MINIROS_EXPORT
    roscxx
    bag_storage
    ${rosbag_INTERNAL_LIBS}
    PARENT_SCOPE)
