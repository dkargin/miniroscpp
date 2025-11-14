set(CMAKE_CXX_STANDARD 17)
include_directories(${MINIROS_INCLUDE_TRANSPORT})

set(TEST_GROUP basic)

miniros_add_test(${TEST_GROUP}-test_version SOURCES test_version.cpp)
miniros_add_test(${TEST_GROUP}-test_header SOURCES test_header.cpp)
miniros_add_test(${TEST_GROUP}-test_poll_set SOURCES test_poll_set.cpp)

if(WIN32)
  target_link_libraries(${TEST_GROUP}-test_poll_set ws2_32)
endif()

miniros_add_test(${TEST_GROUP}-test_transport_tcp SOURCES test_transport_tcp.cpp)
miniros_add_test(${TEST_GROUP}-test_subscription_queue SOURCES test_subscription_queue.cpp)
miniros_add_test(${TEST_GROUP}-test_callback_queue SOURCES test_callback_queue.cpp)
miniros_add_test(${TEST_GROUP}-test_names SOURCES  test_names.cpp)
miniros_add_test(${TEST_GROUP}-test_args SOURCES test_args.cpp)
miniros_add_test(${TEST_GROUP}-test_observer SOURCES test_observer.cpp)
miniros_add_test(${TEST_GROUP}-test_http_parser SOURCES test_http_parser.cpp)
miniros_add_test(${TEST_GROUP}-test_http SOURCES test_http.cpp)
miniros_add_test(${TEST_GROUP}-test_net SOURCES test_net.cpp)

if(catkin_FOUND AND CATKIN_ENABLE_TESTING)
  message(STATUS "Trying to find rostest for more complex testing")
  find_package(rostest REQUIRED)
endif()

if (CATKIN_FOUND AND ROSTEST_FOUND)
  # TODO: keeping this list for reference.
  add_rostest(launch/real_time_test.xml)
  add_rostest(launch/sim_time_test.xml)

  # Publish one message
  add_rostest(launch/pubsub_once.xml)

  # Publish a bunch of messages back to back
  add_rostest(launch/pubsub_n_fast.xml)
  add_rostest(launch/pubsub_n_fast_udp.xml)

  # Publish a bunch of empty messages
  add_rostest(launch/pubsub_empty.xml)

  # Publish only to the subscriber from the subscriber callback
  add_rostest(launch/pub_onsub.xml)

  # Publish a bunch of large messages back to back
  add_rostest(launch/pubsub_n_fast_large_message.xml)

  # Subscribe, listen, unsubscribe, re-subscribe to a different topic, listen
  # again
  add_rostest(launch/pubsub_resub_once.xml)

  # Subscribe and unsubscribe repeatedly, ensuring that callbacks don't get
  # called when not subscribed.
  add_rostest(launch/pubsub_unsub.xml)

  # Advertise, then unadvertise, and ensure that subscriber callback doesn't
  # get invoked afterward, while a subscriber is constantly subscribing and
  # unsubscribing
  add_rostest(launch/pubsub_unadv.xml)

  # Call a service
  add_rostest(launch/service_call.xml)

  # disabling the test again since it does not work realiable
  #add_rostest(launch/service_deadlock.xml)

  add_rostest(launch/service_exception.xml)

  add_rostest(launch/service_call_unadv.xml)

  add_rostest(launch/service_call_zombie.xml)

  # Repeatedly call miniros::init() and miniros::fini()
  add_rostest(launch/multiple_init_fini.xml)

  # Test node inspection functionality
  add_rostest(launch/inspection.xml)

  # Test that advertising a service multiple times fails
  add_rostest(launch/service_adv_multiple.xml)

  # Test that the second node to advertise a service "wins"
  add_rostest(launch/service_multiple_providers.xml)

  # Test namespaces
  add_rostest(launch/namespaces.xml)

  # Test command-line name remapping
  add_rostest(launch/name_remapping.xml)

  add_rostest(launch/name_not_remappable.xml)

  # Test command-line name remapping
  add_rostest(launch/name_remapping_ROS_NAMESPACE.xml)

  # Test params
  add_rostest(launch/params.xml)

  # Test getting information from the master
  add_rostest(launch/get_master_information.xml)

  # Test multiple subscriptions
  add_rostest(launch/multiple_subscriptions.xml)

  add_rostest(launch/pingpong.xml)
  add_rostest(launch/pingpong_large.xml)

  add_rostest(launch/subscribe_self.xml)

  add_rostest(launch/check_master.xml)
  #add_rostest(launch/check_master_false.xml)

  add_rostest(launch/handles.xml)
  add_rostest(launch/timer_callbacks.xml)
  add_rostest(launch/latching_publisher.xml)
  add_rostest(launch/loads_of_publishers.xml)
  add_rostest(launch/incrementing_sequence.xml)
  add_rostest(launch/subscription_callback_types.xml)
  add_rostest(launch/service_callback_types.xml)
  add_rostest(launch/intraprocess_subscriptions.xml)
  add_rostest(launch/nonconst_subscriptions.xml)
  add_rostest(launch/subscribe_retry_tcp.xml)
  add_rostest(launch/subscribe_star.xml)
  add_rostest(launch/parameter_validation.xml)

  add_rostest(launch/no_remappings.xml)
  add_rostest(launch/local_remappings.xml)
  add_rostest(launch/global_remappings.xml)
  add_rostest(launch/ns_node_remapping.xml)
  add_rostest(launch/search_param.xml)

  add_rostest(launch/stamped_topic_statistics_with_empty_timestamp.xml)
  add_rostest(launch/topic_statistic_frequency.xml DEPENDENCIES ${TEST_GROUP}-publisher_rate ${TEST_GROUP}-subscriber ${TEST_GROUP}-topic_statistic_frequency)

  # Test spinners
  add_rostest(launch/spinners.xml)
endif()