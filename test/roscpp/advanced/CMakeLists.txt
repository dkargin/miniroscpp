include_directories(${GTEST_INCLUDE_DIRS})
message(STATUS "Generated includes: ${MINIROS_GENERATED_INCLUDE_DIRS}")
include_directories(${MINIROS_GENERATED_INCLUDE_DIRS})

link_directories(${GTEST_LIBRARY_DIRS})

set(TEST_GROUP advanced)

miniros_add_test(${TEST_GROUP}-handles SOURCES handles.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-timer_callbacks SOURCES timer_callbacks.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-latching_publisher SOURCES latching_publisher.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-publish_n_fast SOURCES publish_n_fast.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-subscribe_self SOURCES subscribe_self.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-pub_sub SOURCES pub_sub.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-sub_pub SOURCES sub_pub.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-publish_empty SOURCES publish_empty.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-publish_onsub SOURCES publish_onsub.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-subscribe_n_fast SOURCES subscribe_n_fast.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-subscribe_empty SOURCES subscribe_empty.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-subscribe_resubscribe SOURCES subscribe_resubscribe.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-subscribe_unsubscribe SOURCES subscribe_unsubscribe.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-publish_unadvertise SOURCES publish_unadvertise.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-subscribe_unsubscribe_repeatedly SOURCES subscribe_unsubscribe_repeatedly.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-publish_constantly SOURCES publish_constantly.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-param_update_test SOURCES param_update_test.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-real_time_test SOURCES real_time_test.cpp WITH_MASTER DISABLED)
#add_dependencies(${TEST_GROUP}-real_time_test ${rosgraph_msgs_EXPORTED_TARGETS})

miniros_add_test(${TEST_GROUP}-sim_time_test SOURCES sim_time_test.cpp WITH_MASTER DISABLED)
#add_dependencies(${TEST_GROUP}-sim_time_test ${rosgraph_msgs_EXPORTED_TARGETS})

miniros_add_test(${TEST_GROUP}-init_no_sim_time_test SOURCES init_no_sim_time_test.cpp WITH_MASTER DISABLED)

# Call a service
miniros_add_test(${TEST_GROUP}-service_adv SOURCES service_adv.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-service_adv_unadv SOURCES service_adv_unadv.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-service_call SOURCES service_call.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-service_call_zombie SOURCES service_call_zombie.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-service_deadlock SOURCES service_deadlock.cpp WITH_MASTER DISABLED)
#add_dependencies(${TEST_GROUP}-service_deadlock ${std_srvs_EXPORTED_TARGETS})

miniros_add_test(${TEST_GROUP}-service_call_repeatedly SOURCES service_call_repeatedly.cpp WITH_MASTER DISABLED)

# No support for Log4xx
#miniros_add_test(${TEST_GROUP}-service_exception SOURCES service_exception.cpp)

# Repeatedly call miniros::init()
miniros_add_test(${TEST_GROUP}-multiple_init_fini SOURCES multiple_init_fini.cpp WITH_MASTER DISABLED)

# This one tries to use exported boolean variable that no longer exits.
# miniros_add_test(${TEST_GROUP}-missing_call_to_shutdown_impl SOURCES missing_call_to_shutdown_impl.cpp)

miniros_add_test(${TEST_GROUP}-missing_call_to_shutdown SOURCES missing_call_to_shutdown.cpp WITH_MASTER DISABLED)

# Test node inspection functionality
miniros_add_test(${TEST_GROUP}-inspection SOURCES inspection.cpp WITH_MASTER DISABLED)

# Test that advertising a service multiple times fails
miniros_add_test(${TEST_GROUP}-service_adv_multiple SOURCES service_adv_multiple.cpp WITH_MASTER DISABLED)

# Test that the second node to advertise a service "wins"
miniros_add_test(${TEST_GROUP}-service_adv_a SOURCES service_adv_a.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-service_wait_a_adv_b SOURCES service_wait_a_adv_b.cpp WITH_MASTER DISABLED)

# Zombie (node crashed) service
add_executable(${TEST_GROUP}-service_adv_zombie service_adv_zombie.cpp)
target_link_libraries(${TEST_GROUP}-service_adv_zombie roscxx ${GTEST_LIBRARIES})

add_executable(${TEST_GROUP}-service_call_expect_b service_call_expect_b.cpp)
target_link_libraries(${TEST_GROUP}-service_call_expect_b roscxx ${GTEST_LIBRARIES})

# Test command-line name remapping
miniros_add_test(${TEST_GROUP}-name_remapping SOURCES name_remapping.cpp WITH_MASTER DISABLED)
miniros_add_test(${TEST_GROUP}-name_remapping_with_ns SOURCES name_remapping_with_ns.cpp WITH_MASTER DISABLED)

# Test namespaces
miniros_add_test(${TEST_GROUP}-namespaces SOURCES namespaces.cpp WITH_MASTER DISABLED)

# Test params
miniros_add_test(${TEST_GROUP}-params SOURCES params.cpp WITH_MASTER DISABLED)

# Test spinners
miniros_add_test(${TEST_GROUP}-spinners SOURCES spinners.cpp WITH_MASTER DISABLED)

# Test getting information from the master
miniros_add_test(${TEST_GROUP}-get_master_information SOURCES get_master_information.cpp WITH_MASTER DISABLED)

# Test multiple subscriptions
miniros_add_test(${TEST_GROUP}-multiple_subscriptions SOURCES multiple_subscriptions.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-check_master SOURCES check_master.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-wait_for_message SOURCES wait_for_message.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-loads_of_publishers SOURCES loads_of_publishers.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-incrementing_sequence SOURCES incrementing_sequence.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-subscription_callback_types SOURCES subscription_callback_types.cpp WITH_MASTER DISABLED)
#add_dependencies(${TEST_GROUP}-subscription_callback_types ${std_msgs_EXPORTED_TARGETS})

miniros_add_test(${TEST_GROUP}-service_callback_types SOURCES service_callback_types.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-intraprocess_subscriptions SOURCES intraprocess_subscriptions.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-nonconst_subscriptions SOURCES nonconst_subscriptions.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-subscribe_retry_tcp SOURCES subscribe_retry_tcp.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-subscribe_star SOURCES subscribe_star.cpp WITH_MASTER DISABLED)
#add_dependencies(${TEST_GROUP}-subscribe_star ${std_srvs_EXPORTED_TARGETS})

miniros_add_test(${TEST_GROUP}-publisher_for_star_subscriber SOURCES publisher_for_star_subscriber.cpp WITH_MASTER DISABLED)
#add_dependencies(${TEST_GROUP}-publisher_for_star_subscriber ${std_srvs_EXPORTED_TARGETS})

miniros_add_test(${TEST_GROUP}-parameter_validation SOURCES parameter_validation.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-param_locale_avoidance_test SOURCES param_locale_avoidance_test.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-crashes_under_gprof SOURCES crashes_under_gprof.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-test_remapping SOURCES test_remapping.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-name_not_remappable SOURCES name_not_remappable.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-test_ns_node_remapping SOURCES test_ns_node_remapping.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-test_search_param SOURCES test_search_param.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-left_right SOURCES left_right.cpp WITH_MASTER DISABLED)
#add_dependencies(${TEST_GROUP}-left_right ${std_msgs_EXPORTED_TARGETS})

miniros_add_test(${TEST_GROUP}-string_msg_expect SOURCES string_msg_expect.cpp WITH_MASTER DISABLED)
#add_dependencies(${TEST_GROUP}-string_msg_expect ${std_msgs_EXPORTED_TARGETS})

miniros_add_test(${TEST_GROUP}-stamped_topic_statistics_empty_timestamp SOURCES stamped_topic_statistics_empty_timestamp.cpp WITH_MASTER DISABLED)

miniros_add_test(${TEST_GROUP}-topic_statistic_frequency SOURCES topic_statistic_frequency.cpp WITH_MASTER DISABLED)

# This test requires ability to publish primitive "bool" type, which does not work now.
# But I do not see a big reason to support it.
#miniros_add_test(${TEST_GROUP}-multiple_latched_publishers SOURCES multiple_latched_publishers.cpp WITH_MASTER  DISABLED)
#add_dependencies(${TEST_GROUP}-multiple_latched_publishers ${std_msgs_EXPORTED_TARGETS})

# The publishers and subscriber are compiled but not registered as a unit test
# since the test execution requires a network connection which drops packages.
# Call scripts/test_udp_with_dropped_packets.sh to run the test.
miniros_add_test(${TEST_GROUP}-publisher SOURCES publisher.cpp WITH_MASTER DISABLED)
#add_dependencies(${TEST_GROUP}-publisher ${std_msgs_EXPORTED_TARGETS})

miniros_add_test(${TEST_GROUP}-publisher_rate SOURCES publisher_rate.cpp WITH_MASTER DISABLED)
#add_dependencies(${TEST_GROUP}-publisher_rate ${std_msgs_EXPORTED_TARGETS})

miniros_add_test(${TEST_GROUP}-subscriber SOURCES subscriber.cpp WITH_MASTER DISABLED)
#add_dependencies(${TEST_GROUP}-subscriber ${std_msgs_EXPORTED_TARGETS})

