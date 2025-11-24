include_directories(${MINIROS_INCLUDE_ROOT})
# This path is needed for xmlrpc includes
include_directories(${MINIROS_INCLUDE_ROOT}/miniros)

miniros_add_test(xmlrpcvalue_base64 SOURCES xmlrpcvalue_base64.cpp)

miniros_add_test(test_base64 SOURCES test_base64.cpp)

if(WIN32)
  # On Windows, gtest can be built as shared (dll) or static (lib),
  # to simplify the problem, here we require to match BUILD_SHARED_LIBS we used
  if(BUILD_SHARED_LIBS)
    set_target_properties(test_base64 PROPERTIES COMPILE_DEFINITIONS "GTEST_LINKED_AS_SHARED_LIBRARY=1")
  endif()
endif()

add_library(test_fixtures STATIC test_fixtures.cpp)
target_link_libraries(test_fixtures roscxx ${GTEST_LIBRARIES})
set_target_properties(test_fixtures PROPERTIES EXCLUDE_FROM_ALL TRUE)

miniros_add_test(HelloTest SOURCES HelloTest.cpp)

miniros_add_test(test_dispatch_live SOURCES test_dispatch_live.cpp)
target_link_libraries(test_dispatch_live test_fixtures)

miniros_add_test(test_ulimit SOURCES test_ulimit.cpp)
target_link_libraries(test_ulimit test_fixtures)

add_library(mock_socket STATIC mock_socket.cpp)
target_link_libraries(mock_socket ${GTEST_LIBRARIES})
set_target_properties(mock_socket PROPERTIES EXCLUDE_FROM_ALL TRUE)

# Some tests rely on replacing system calls (like write/read) by a mock versions.
# But we must include actual sources to make this replacement happen.
miniros_add_test(test_client SOURCES test_client.cpp
  ${XMLRPC_SRC_ROOT}/XmlRpcClient.cpp
  ${XMLRPC_SRC_ROOT}/XmlRpcValue.cpp
  ${XMLRPC_SRC_ROOT}/XmlRpcUtil.cpp
  ${XMLRPC_SRC_ROOT}/XmlRpcDispatch.cpp
  ${XMLRPC_SRC_ROOT}/XmlRpcSource.cpp
  ${XMLRPC_SRC_ROOT}/../b64/b64decode.cpp
  ${XMLRPC_SRC_ROOT}/../b64/b64encode.cpp)

target_link_libraries(test_client mock_socket)
if(WIN32)
  target_link_libraries(test_client ws2_32)
endif()

if(WIN32)
  # build everything from source, undefine MINIROS_BUILD_SHARED_LIBS to eliminate the dllimport/dllexport attributes
  set_target_properties(test_client PROPERTIES COMPILE_FLAGS -UMINIROS_BUILD_SHARED_LIBS)
endif()

miniros_add_test(test_dispatch SOURCES test_dispatch.cpp
  ${XMLRPC_SRC_ROOT}/XmlRpcDispatch.cpp
  ${XMLRPC_SRC_ROOT}/XmlRpcSource.cpp
  ${XMLRPC_SRC_ROOT}/XmlRpcUtil.cpp
  ${XMLRPC_SRC_ROOT}/../b64/b64decode.cpp
  ${XMLRPC_SRC_ROOT}/../b64/b64encode.cpp)

if(WIN32)
  # build everything from source, undefine MINIROS_BUILD_SHARED_LIBS to eliminate the dllimport/dllexport attributes
  set_target_properties(test_dispatch PROPERTIES COMPILE_FLAGS -UMINIROS_BUILD_SHARED_LIBS)
endif()

target_link_libraries(test_dispatch mock_socket)
if(WIN32)
  target_link_libraries(test_dispatch ws2_32)
endif()

if(APPLE)
  set_target_properties(test_dispatch PROPERTIES LINK_FLAGS "-Wl,-alias,___wrap_poll,_poll")
elseif(UNIX)
  set_target_properties(test_dispatch PROPERTIES LINK_FLAGS "-Wl,--wrap=poll")
endif()


if(NOT WIN32)
  add_executable(test_socket test_socket.cpp test_system_mocks.c
    ${XMLRPC_SRC_ROOT}/XmlRpcSocket.cpp
    ${XMLRPC_SRC_ROOT}/XmlRpcUtil.cpp
  )

  if(APPLE)
    set_target_properties(test_socket PROPERTIES
      LINK_FLAGS
      "-Wl,-alias,___wrap_accept,_accept -Wl,-alias,___wrap_bind,_bind -Wl,-alias,___wrap_close,_close -Wl,-alias,___wrap_connect,_connect -Wl,-alias,___wrap_getaddrinfo,_getaddrinfo -Wl,-alias,___wrap_getsockname,_getsockname -Wl,-alias,___wrap_listen,_listen -Wl,-alias,___wrap_read,_read -Wl,-alias,___wrap_setsockopt,_setsockopt -Wl,-alias,___wrap_select,_select -Wl,-alias,___wrap_select,_select$1050 -Wl,-alias,___wrap_socket,_socket -Wl,-alias,___wrap_write,_write -Wl,-alias,___wrap_fcntl,_fcntl -Wl,-alias,___wrap_freeaddrinfo,_freeaddrinfo"
    )
  elseif(UNIX)
    set_target_properties(test_socket PROPERTIES
      LINK_FLAGS
      "-Wl,--wrap=accept -Wl,--wrap=bind -Wl,--wrap=close -Wl,--wrap=connect -Wl,--wrap=getaddrinfo -Wl,--wrap=getsockname -Wl,--wrap=listen -Wl,--wrap=read -Wl,--wrap=setsockopt -Wl,--wrap=select -Wl,--wrap=socket -Wl,--wrap=write -Wl,--wrap=fcntl -Wl,--wrap=freeaddrinfo"
    )
    target_link_libraries(test_socket pthread)
  endif()
  target_link_libraries(test_socket ${GTEST_LIBRARIES})
  add_test(NAME test_socket COMMAND test_socket)
endif()

miniros_add_test(TestValues SOURCES TestValues.cpp)

miniros_add_test(test_util SOURCES test_util.cpp)

miniros_add_test(TestXml SOURCES TestXml.cpp)
