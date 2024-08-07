if(NOT WIN32)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")
endif()

include_directories(
  # This thing can ocasionly override local paths
  #${MINIROS_INCLUDE_ROOT}
  libb64/include
)

if(WIN32)
  add_definitions(-D_WINDOWS)
endif()

add_library(xmlrpcpp
  src/XmlRpcClient.cpp
  src/XmlRpcDispatch.cpp
  src/XmlRpcServer.cpp
  src/XmlRpcServerConnection.cpp
  src/XmlRpcServerMethod.cpp
  src/XmlRpcSocket.cpp
  src/XmlRpcSource.cpp
  src/XmlRpcUtil.cpp
  src/XmlRpcValue.cpp
  libb64/src/cdecode.c
  libb64/src/cencode.c
)

set_target_properties(xmlrpcpp PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Allows access to generated header miniros/config.h
target_include_directories(xmlrpcpp PRIVATE AFTER ${MINIROS_INCLUDE_DIRS} ${MINIROS_INCLUDE_ROOT})

target_link_libraries(xmlrpcpp miniros::time)

if(WIN32)
  target_link_libraries(xmlrpcpp ws2_32)
endif()


#install(DIRECTORY include/${PROJECT_NAME}
#  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h")

#install(TARGETS xmlrpcpp
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

#if (CATKIN_ENABLE_TESTING)
#  add_subdirectory(test)
#endif()
