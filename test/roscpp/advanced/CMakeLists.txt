include_directories(${GTEST_INCLUDE_DIRS})
message(STATUS "Generated includes: ${MINIROS_GENERATED_INCLUDE_DIRS}")
include_directories(${MINIROS_GENERATED_INCLUDE_DIRS})

link_directories(${GTEST_LIBRARY_DIRS})

set(TEST_GROUP advanced)

miniros_add_test(${TEST_GROUP}-handles EXCLUDE_FROM_ALL handles.cpp)

miniros_add_test(${TEST_GROUP}-timer_callbacks EXCLUDE_FROM_ALL timer_callbacks.cpp)

miniros_add_test(${TEST_GROUP}-latching_publisher EXCLUDE_FROM_ALL latching_publisher.cpp)

miniros_add_test(${TEST_GROUP}-publish_n_fast EXCLUDE_FROM_ALL publish_n_fast.cpp)

miniros_add_test(${TEST_GROUP}-subscribe_self EXCLUDE_FROM_ALL subscribe_self.cpp)

miniros_add_test(${TEST_GROUP}-pub_sub EXCLUDE_FROM_ALL pub_sub.cpp)

miniros_add_test(${TEST_GROUP}-sub_pub EXCLUDE_FROM_ALL sub_pub.cpp)

miniros_add_test(${TEST_GROUP}-publish_empty EXCLUDE_FROM_ALL publish_empty.cpp)

miniros_add_test(${TEST_GROUP}-publish_onsub EXCLUDE_FROM_ALL publish_onsub.cpp)

miniros_add_test(${TEST_GROUP}-subscribe_n_fast EXCLUDE_FROM_ALL subscribe_n_fast.cpp)

miniros_add_test(${TEST_GROUP}-subscribe_empty EXCLUDE_FROM_ALL subscribe_empty.cpp)

miniros_add_test(${TEST_GROUP}-subscribe_resubscribe EXCLUDE_FROM_ALL subscribe_resubscribe.cpp)

miniros_add_test(${TEST_GROUP}-subscribe_unsubscribe EXCLUDE_FROM_ALL subscribe_unsubscribe.cpp)

miniros_add_test(${TEST_GROUP}-publish_unadvertise EXCLUDE_FROM_ALL publish_unadvertise.cpp)

miniros_add_test(${TEST_GROUP}-subscribe_unsubscribe_repeatedly EXCLUDE_FROM_ALL subscribe_unsubscribe_repeatedly.cpp)

miniros_add_test(${TEST_GROUP}-publish_constantly EXCLUDE_FROM_ALL publish_constantly.cpp)

miniros_add_test(${TEST_GROUP}-param_update_test EXCLUDE_FROM_ALL param_update_test.cpp)

miniros_add_test(${TEST_GROUP}-real_time_test EXCLUDE_FROM_ALL real_time_test.cpp)
#add_dependencies(${TEST_GROUP}-real_time_test ${rosgraph_msgs_EXPORTED_TARGETS})

miniros_add_test(${TEST_GROUP}-sim_time_test EXCLUDE_FROM_ALL sim_time_test.cpp)
#add_dependencies(${TEST_GROUP}-sim_time_test ${rosgraph_msgs_EXPORTED_TARGETS})

miniros_add_test(${TEST_GROUP}-init_no_sim_time_test EXCLUDE_FROM_ALL init_no_sim_time_test.cpp)

# Call a service
miniros_add_test(${TEST_GROUP}-service_adv EXCLUDE_FROM_ALL service_adv.cpp)

miniros_add_test(${TEST_GROUP}-service_adv_unadv EXCLUDE_FROM_ALL service_adv_unadv.cpp)

miniros_add_test(${TEST_GROUP}-service_call EXCLUDE_FROM_ALL service_call.cpp)

miniros_add_test(${TEST_GROUP}-service_call_zombie EXCLUDE_FROM_ALL service_call_zombie.cpp)

miniros_add_test(${TEST_GROUP}-service_deadlock EXCLUDE_FROM_ALL service_deadlock.cpp)
#add_dependencies(${TEST_GROUP}-service_deadlock ${std_srvs_EXPORTED_TARGETS})

miniros_add_test(${TEST_GROUP}-service_call_repeatedly EXCLUDE_FROM_ALL service_call_repeatedly.cpp)

# No support for Log4xx
#miniros_add_test(${TEST_GROUP}-service_exception EXCLUDE_FROM_ALL service_exception.cpp)

# Repeatedly call miniros::init()
miniros_add_test(${TEST_GROUP}-multiple_init_fini EXCLUDE_FROM_ALL multiple_init_fini.cpp)

# This one tries to use exported boolean variable that no longer exits.
# miniros_add_test(${TEST_GROUP}-missing_call_to_shutdown_impl EXCLUDE_FROM_ALL missing_call_to_shutdown_impl.cpp)

miniros_add_test(${TEST_GROUP}-missing_call_to_shutdown EXCLUDE_FROM_ALL missing_call_to_shutdown.cpp)

# Test node inspection functionality
miniros_add_test(${TEST_GROUP}-inspection EXCLUDE_FROM_ALL inspection.cpp)

# Test that advertising a service multiple times fails
miniros_add_test(${TEST_GROUP}-service_adv_multiple EXCLUDE_FROM_ALL service_adv_multiple.cpp)

# Test that the second node to advertise a service "wins"
miniros_add_test(${TEST_GROUP}-service_adv_a EXCLUDE_FROM_ALL service_adv_a.cpp)

miniros_add_test(${TEST_GROUP}-service_wait_a_adv_b EXCLUDE_FROM_ALL service_wait_a_adv_b.cpp)

# Zombie (node crashed) service
add_executable(${TEST_GROUP}-service_adv_zombie EXCLUDE_FROM_ALL service_adv_zombie.cpp)
target_link_libraries(${TEST_GROUP}-service_adv_zombie miniros::roscxx ${GTEST_LIBRARIES})

add_executable(${TEST_GROUP}-service_call_expect_b EXCLUDE_FROM_ALL service_call_expect_b.cpp)
target_link_libraries(${TEST_GROUP}-service_call_expect_b miniros::roscxx ${GTEST_LIBRARIES})

# Test command-line name remapping
miniros_add_test(${TEST_GROUP}-name_remapping EXCLUDE_FROM_ALL name_remapping.cpp)
miniros_add_test(${TEST_GROUP}-name_remapping_with_ns EXCLUDE_FROM_ALL name_remapping_with_ns.cpp)

# Test namespaces
miniros_add_test(${TEST_GROUP}-namespaces EXCLUDE_FROM_ALL namespaces.cpp)

# Test params
miniros_add_test(${TEST_GROUP}-params EXCLUDE_FROM_ALL params.cpp)

# Test spinners
miniros_add_test(${TEST_GROUP}-spinners EXCLUDE_FROM_ALL spinners.cpp)

# Test getting information from the master
miniros_add_test(${TEST_GROUP}-get_master_information EXCLUDE_FROM_ALL get_master_information.cpp)

# Test multiple subscriptions
miniros_add_test(${TEST_GROUP}-multiple_subscriptions EXCLUDE_FROM_ALL multiple_subscriptions.cpp)

miniros_add_test(${TEST_GROUP}-check_master EXCLUDE_FROM_ALL check_master.cpp)

miniros_add_test(${TEST_GROUP}-wait_for_message EXCLUDE_FROM_ALL wait_for_message.cpp)

miniros_add_test(${TEST_GROUP}-loads_of_publishers EXCLUDE_FROM_ALL loads_of_publishers.cpp)

miniros_add_test(${TEST_GROUP}-incrementing_sequence EXCLUDE_FROM_ALL incrementing_sequence.cpp)

miniros_add_test(${TEST_GROUP}-subscription_callback_types EXCLUDE_FROM_ALL subscription_callback_types.cpp)
#add_dependencies(${TEST_GROUP}-subscription_callback_types ${std_msgs_EXPORTED_TARGETS})

miniros_add_test(${TEST_GROUP}-service_callback_types EXCLUDE_FROM_ALL service_callback_types.cpp)

miniros_add_test(${TEST_GROUP}-intraprocess_subscriptions EXCLUDE_FROM_ALL intraprocess_subscriptions.cpp)

miniros_add_test(${TEST_GROUP}-nonconst_subscriptions EXCLUDE_FROM_ALL nonconst_subscriptions.cpp)

miniros_add_test(${TEST_GROUP}-subscribe_retry_tcp EXCLUDE_FROM_ALL subscribe_retry_tcp.cpp)

miniros_add_test(${TEST_GROUP}-subscribe_star EXCLUDE_FROM_ALL subscribe_star.cpp)
#add_dependencies(${TEST_GROUP}-subscribe_star ${std_srvs_EXPORTED_TARGETS})

miniros_add_test(${TEST_GROUP}-publisher_for_star_subscriber EXCLUDE_FROM_ALL publisher_for_star_subscriber.cpp)
#add_dependencies(${TEST_GROUP}-publisher_for_star_subscriber ${std_srvs_EXPORTED_TARGETS})

miniros_add_test(${TEST_GROUP}-parameter_validation EXCLUDE_FROM_ALL parameter_validation.cpp)

miniros_add_test(${TEST_GROUP}-param_locale_avoidance_test EXCLUDE_FROM_ALL param_locale_avoidance_test.cpp)

miniros_add_test(${TEST_GROUP}-crashes_under_gprof EXCLUDE_FROM_ALL crashes_under_gprof.cpp)

miniros_add_test(${TEST_GROUP}-test_remapping EXCLUDE_FROM_ALL test_remapping.cpp)

miniros_add_test(${TEST_GROUP}-name_not_remappable EXCLUDE_FROM_ALL name_not_remappable.cpp)

miniros_add_test(${TEST_GROUP}-test_ns_node_remapping EXCLUDE_FROM_ALL test_ns_node_remapping.cpp)

miniros_add_test(${TEST_GROUP}-test_search_param EXCLUDE_FROM_ALL test_search_param.cpp)

miniros_add_test(${TEST_GROUP}-left_right EXCLUDE_FROM_ALL left_right.cpp)
#add_dependencies(${TEST_GROUP}-left_right ${std_msgs_EXPORTED_TARGETS})

miniros_add_test(${TEST_GROUP}-string_msg_expect EXCLUDE_FROM_ALL string_msg_expect.cpp)
#add_dependencies(${TEST_GROUP}-string_msg_expect ${std_msgs_EXPORTED_TARGETS})

miniros_add_test(${TEST_GROUP}-stamped_topic_statistics_empty_timestamp EXCLUDE_FROM_ALL stamped_topic_statistics_empty_timestamp.cpp)

miniros_add_test(${TEST_GROUP}-topic_statistic_frequency EXCLUDE_FROM_ALL topic_statistic_frequency.cpp)

# This test requires ability to publish primitive "bool" type, which does not work now.
# But I do not see a big reason to support it.
#miniros_add_test(${TEST_GROUP}-multiple_latched_publishers EXCLUDE_FROM_ALL multiple_latched_publishers.cpp)
#add_dependencies(${TEST_GROUP}-multiple_latched_publishers ${std_msgs_EXPORTED_TARGETS})

# The publishers and subscriber are compiled but not registered as a unit test
# since the test execution requires a network connection which drops packages.
# Call scripts/test_udp_with_dropped_packets.sh to run the test.
miniros_add_test(${TEST_GROUP}-publisher EXCLUDE_FROM_ALL publisher.cpp)
#add_dependencies(${TEST_GROUP}-publisher ${std_msgs_EXPORTED_TARGETS})

miniros_add_test(${TEST_GROUP}-publisher_rate EXCLUDE_FROM_ALL publisher_rate.cpp)
#add_dependencies(${TEST_GROUP}-publisher_rate ${std_msgs_EXPORTED_TARGETS})

miniros_add_test(${TEST_GROUP}-subscriber EXCLUDE_FROM_ALL subscriber.cpp)
#add_dependencies(${TEST_GROUP}-subscriber ${std_msgs_EXPORTED_TARGETS})

if(TARGET tests)
  add_dependencies(tests
    ${TEST_GROUP}-handles
    ${TEST_GROUP}-timer_callbacks
    ${TEST_GROUP}-latching_publisher
    ${TEST_GROUP}-publish_n_fast
    ${TEST_GROUP}-subscribe_self
    ${TEST_GROUP}-pub_sub
    ${TEST_GROUP}-sub_pub
    ${TEST_GROUP}-publish_empty
    ${TEST_GROUP}-publish_onsub
    ${TEST_GROUP}-subscribe_n_fast
    ${TEST_GROUP}-subscribe_empty
    ${TEST_GROUP}-subscribe_resubscribe
    ${TEST_GROUP}-subscribe_unsubscribe
    ${TEST_GROUP}-publish_unadvertise
    ${TEST_GROUP}-subscribe_unsubscribe_repeatedly
    ${TEST_GROUP}-publish_constantly
    ${TEST_GROUP}-param_update_test
    ${TEST_GROUP}-real_time_test
    ${TEST_GROUP}-sim_time_test
    ${TEST_GROUP}-service_adv
    ${TEST_GROUP}-service_adv_unadv
    ${TEST_GROUP}-service_call
    ${TEST_GROUP}-service_call_zombie
    ${TEST_GROUP}-service_deadlock
#    ${TEST_GROUP}-service_exception
    ${TEST_GROUP}-service_call_repeatedly
    ${TEST_GROUP}-multiple_init_fini
    ${TEST_GROUP}-missing_call_to_shutdown
#    ${TEST_GROUP}-missing_call_to_shutdown_impl
    ${TEST_GROUP}-inspection
    ${TEST_GROUP}-service_adv_multiple
    ${TEST_GROUP}-service_adv_a
    ${TEST_GROUP}-service_adv_zombie
    ${TEST_GROUP}-service_wait_a_adv_b
    ${TEST_GROUP}-service_call_expect_b
    ${TEST_GROUP}-name_remapping
    ${TEST_GROUP}-name_remapping_with_ns
    ${TEST_GROUP}-namespaces
    ${TEST_GROUP}-params
    ${TEST_GROUP}-spinners
    ${TEST_GROUP}-get_master_information
    ${TEST_GROUP}-multiple_subscriptions
    ${TEST_GROUP}-check_master
    ${TEST_GROUP}-wait_for_message
    ${TEST_GROUP}-loads_of_publishers
    ${TEST_GROUP}-incrementing_sequence
    ${TEST_GROUP}-subscription_callback_types
    ${TEST_GROUP}-service_callback_types
    ${TEST_GROUP}-intraprocess_subscriptions
    ${TEST_GROUP}-nonconst_subscriptions
    ${TEST_GROUP}-subscribe_retry_tcp
    ${TEST_GROUP}-subscribe_star
    ${TEST_GROUP}-publisher_for_star_subscriber
    ${TEST_GROUP}-parameter_validation
    ${TEST_GROUP}-param_locale_avoidance_test
    ${TEST_GROUP}-crashes_under_gprof
    ${TEST_GROUP}-test_remapping
    ${TEST_GROUP}-name_not_remappable
    ${TEST_GROUP}-test_ns_node_remapping
    ${TEST_GROUP}-test_search_param
    ${TEST_GROUP}-left_right
    ${TEST_GROUP}-string_msg_expect
    ${TEST_GROUP}-publisher
    ${TEST_GROUP}-subscriber
    ${TEST_GROUP}-stamped_topic_statistics_empty_timestamp
    ${TEST_GROUP}-topic_statistic_frequency
#    ${TEST_GROUP}-multiple_latched_publishers
  )
endif()
