include_directories(${MINIROS_GENERATED_INCLUDE_DIRS})

set(MULTIMASTER_TEST_BIN_DIR "${CMAKE_BINARY_DIR}/bin")

add_executable(mm_requester mm_requester.cpp)
target_link_libraries(mm_requester roscxx)
target_include_directories(mm_requester PRIVATE ${MINIROS_GENERATED_INCLUDE_DIRS})
set_target_properties(mm_requester PROPERTIES FOLDER "Tests/Multimaster")

add_executable(mm_responder mm_responder.cpp)
target_link_libraries(mm_responder roscxx)
target_include_directories(mm_responder PRIVATE ${MINIROS_GENERATED_INCLUDE_DIRS})
set_target_properties(mm_responder PROPERTIES FOLDER "Tests/Multimaster")

add_executable(mm_publisher mm_publisher.cpp)
target_link_libraries(mm_publisher roscxx)
target_include_directories(mm_publisher PRIVATE ${MINIROS_GENERATED_INCLUDE_DIRS})
set_target_properties(mm_publisher PROPERTIES FOLDER "Tests/Multimaster")

add_executable(mm_subscriber mm_subscriber.cpp)
target_link_libraries(mm_subscriber roscxx)
target_include_directories(mm_subscriber PRIVATE ${MINIROS_GENERATED_INCLUDE_DIRS})
set_target_properties(mm_subscriber PROPERTIES FOLDER "Tests/Multimaster")

add_executable(multimaster_integration_test multimaster_integration_test.cpp)
target_link_libraries(multimaster_integration_test roscxx ${GTEST_LIBRARIES})
target_compile_definitions(multimaster_integration_test PRIVATE
  MINIROS_TEST_BIN_DIR="${MULTIMASTER_TEST_BIN_DIR}")
add_dependencies(multimaster_integration_test
  miniroscore mm_requester mm_responder mm_publisher mm_subscriber)
set_target_properties(multimaster_integration_test PROPERTIES FOLDER "Tests/Multimaster")

# Run directly (not via gdb wrapper) — this test spawns child processes.
add_test(NAME multimaster-integration COMMAND $<TARGET_FILE:multimaster_integration_test>)
