include_directories(${GTEST_INCLUDE_DIRS})
message(STATUS "Generated includes: ${MINIROS_GENERATED_INCLUDE_DIRS}")
include_directories(${MINIROS_GENERATED_INCLUDE_DIRS})

link_directories(${GTEST_LIBRARY_DIRS})

miniros_add_test(${PROJECT_NAME}-handles EXCLUDE_FROM_ALL handles.cpp)

miniros_add_test(${PROJECT_NAME}-timer_callbacks EXCLUDE_FROM_ALL timer_callbacks.cpp)

miniros_add_test(${PROJECT_NAME}-latching_publisher EXCLUDE_FROM_ALL latching_publisher.cpp)

miniros_add_test(${PROJECT_NAME}-publish_n_fast EXCLUDE_FROM_ALL publish_n_fast.cpp)

miniros_add_test(${PROJECT_NAME}-subscribe_self EXCLUDE_FROM_ALL subscribe_self.cpp)

miniros_add_test(${PROJECT_NAME}-pub_sub EXCLUDE_FROM_ALL pub_sub.cpp)

miniros_add_test(${PROJECT_NAME}-sub_pub EXCLUDE_FROM_ALL sub_pub.cpp)

miniros_add_test(${PROJECT_NAME}-publish_empty EXCLUDE_FROM_ALL publish_empty.cpp)

miniros_add_test(${PROJECT_NAME}-publish_onsub EXCLUDE_FROM_ALL publish_onsub.cpp)

miniros_add_test(${PROJECT_NAME}-subscribe_n_fast EXCLUDE_FROM_ALL subscribe_n_fast.cpp)

miniros_add_test(${PROJECT_NAME}-subscribe_empty EXCLUDE_FROM_ALL subscribe_empty.cpp)

miniros_add_test(${PROJECT_NAME}-subscribe_resubscribe EXCLUDE_FROM_ALL subscribe_resubscribe.cpp)

miniros_add_test(${PROJECT_NAME}-subscribe_unsubscribe EXCLUDE_FROM_ALL subscribe_unsubscribe.cpp)

miniros_add_test(${PROJECT_NAME}-publish_unadvertise EXCLUDE_FROM_ALL publish_unadvertise.cpp)

miniros_add_test(${PROJECT_NAME}-subscribe_unsubscribe_repeatedly EXCLUDE_FROM_ALL subscribe_unsubscribe_repeatedly.cpp)

miniros_add_test(${PROJECT_NAME}-publish_constantly EXCLUDE_FROM_ALL publish_constantly.cpp)

miniros_add_test(${PROJECT_NAME}-param_update_test EXCLUDE_FROM_ALL param_update_test.cpp)

miniros_add_test(${PROJECT_NAME}-real_time_test EXCLUDE_FROM_ALL real_time_test.cpp)
#add_dependencies(${PROJECT_NAME}-real_time_test ${rosgraph_msgs_EXPORTED_TARGETS})

miniros_add_test(${PROJECT_NAME}-sim_time_test EXCLUDE_FROM_ALL sim_time_test.cpp)
#add_dependencies(${PROJECT_NAME}-sim_time_test ${rosgraph_msgs_EXPORTED_TARGETS})

miniros_add_test(${PROJECT_NAME}-init_no_sim_time_test EXCLUDE_FROM_ALL init_no_sim_time_test.cpp)

# Call a service
miniros_add_test(${PROJECT_NAME}-service_adv EXCLUDE_FROM_ALL service_adv.cpp)

miniros_add_test(${PROJECT_NAME}-service_adv_unadv EXCLUDE_FROM_ALL service_adv_unadv.cpp)

miniros_add_test(${PROJECT_NAME}-service_call EXCLUDE_FROM_ALL service_call.cpp)

miniros_add_test(${PROJECT_NAME}-service_call_zombie EXCLUDE_FROM_ALL service_call_zombie.cpp)

miniros_add_test(${PROJECT_NAME}-service_deadlock EXCLUDE_FROM_ALL service_deadlock.cpp)
#add_dependencies(${PROJECT_NAME}-service_deadlock ${std_srvs_EXPORTED_TARGETS})

miniros_add_test(${PROJECT_NAME}-service_call_repeatedly EXCLUDE_FROM_ALL service_call_repeatedly.cpp)

# No support for Log4xx
#miniros_add_test(${PROJECT_NAME}-service_exception EXCLUDE_FROM_ALL service_exception.cpp)

# Repeatedly call miniros::init()
miniros_add_test(${PROJECT_NAME}-multiple_init_fini EXCLUDE_FROM_ALL multiple_init_fini.cpp)

# This one tries to use exported boolean variable that no longer exits.
# miniros_add_test(${PROJECT_NAME}-missing_call_to_shutdown_impl EXCLUDE_FROM_ALL missing_call_to_shutdown_impl.cpp)

miniros_add_test(${PROJECT_NAME}-missing_call_to_shutdown EXCLUDE_FROM_ALL missing_call_to_shutdown.cpp)

# Test node inspection functionality
miniros_add_test(${PROJECT_NAME}-inspection EXCLUDE_FROM_ALL inspection.cpp)

# Test that advertising a service multiple times fails
miniros_add_test(${PROJECT_NAME}-service_adv_multiple EXCLUDE_FROM_ALL service_adv_multiple.cpp)

# Test that the second node to advertise a service "wins"
miniros_add_test(${PROJECT_NAME}-service_adv_a EXCLUDE_FROM_ALL service_adv_a.cpp)

miniros_add_test(${PROJECT_NAME}-service_wait_a_adv_b EXCLUDE_FROM_ALL service_wait_a_adv_b.cpp)

# Zombie (node crashed) service
add_executable(${PROJECT_NAME}-service_adv_zombie EXCLUDE_FROM_ALL service_adv_zombie.cpp)
target_link_libraries(${PROJECT_NAME}-service_adv_zombie miniros::roscxx ${GTEST_LIBRARIES})

add_executable(${PROJECT_NAME}-service_call_expect_b EXCLUDE_FROM_ALL service_call_expect_b.cpp)
target_link_libraries(${PROJECT_NAME}-service_call_expect_b miniros::roscxx ${GTEST_LIBRARIES})

# Test command-line name remapping
miniros_add_test(${PROJECT_NAME}-name_remapping EXCLUDE_FROM_ALL name_remapping.cpp)
miniros_add_test(${PROJECT_NAME}-name_remapping_with_ns EXCLUDE_FROM_ALL name_remapping_with_ns.cpp)

# Test namespaces
miniros_add_test(${PROJECT_NAME}-namespaces EXCLUDE_FROM_ALL namespaces.cpp)

# Test params
miniros_add_test(${PROJECT_NAME}-params EXCLUDE_FROM_ALL params.cpp)

# Test spinners
miniros_add_test(${PROJECT_NAME}-spinners EXCLUDE_FROM_ALL spinners.cpp)

# Test getting information from the master
miniros_add_test(${PROJECT_NAME}-get_master_information EXCLUDE_FROM_ALL get_master_information.cpp)

# Test multiple subscriptions
miniros_add_test(${PROJECT_NAME}-multiple_subscriptions EXCLUDE_FROM_ALL multiple_subscriptions.cpp)

miniros_add_test(${PROJECT_NAME}-check_master EXCLUDE_FROM_ALL check_master.cpp)

miniros_add_test(${PROJECT_NAME}-wait_for_message EXCLUDE_FROM_ALL wait_for_message.cpp)

miniros_add_test(${PROJECT_NAME}-loads_of_publishers EXCLUDE_FROM_ALL loads_of_publishers.cpp)

miniros_add_test(${PROJECT_NAME}-incrementing_sequence EXCLUDE_FROM_ALL incrementing_sequence.cpp)

miniros_add_test(${PROJECT_NAME}-subscription_callback_types EXCLUDE_FROM_ALL subscription_callback_types.cpp)
#add_dependencies(${PROJECT_NAME}-subscription_callback_types ${std_msgs_EXPORTED_TARGETS})

miniros_add_test(${PROJECT_NAME}-service_callback_types EXCLUDE_FROM_ALL service_callback_types.cpp)

miniros_add_test(${PROJECT_NAME}-intraprocess_subscriptions EXCLUDE_FROM_ALL intraprocess_subscriptions.cpp)

miniros_add_test(${PROJECT_NAME}-nonconst_subscriptions EXCLUDE_FROM_ALL nonconst_subscriptions.cpp)

miniros_add_test(${PROJECT_NAME}-subscribe_retry_tcp EXCLUDE_FROM_ALL subscribe_retry_tcp.cpp)

miniros_add_test(${PROJECT_NAME}-subscribe_star EXCLUDE_FROM_ALL subscribe_star.cpp)
#add_dependencies(${PROJECT_NAME}-subscribe_star ${std_srvs_EXPORTED_TARGETS})

miniros_add_test(${PROJECT_NAME}-publisher_for_star_subscriber EXCLUDE_FROM_ALL publisher_for_star_subscriber.cpp)
#add_dependencies(${PROJECT_NAME}-publisher_for_star_subscriber ${std_srvs_EXPORTED_TARGETS})

miniros_add_test(${PROJECT_NAME}-parameter_validation EXCLUDE_FROM_ALL parameter_validation.cpp)

miniros_add_test(${PROJECT_NAME}-param_locale_avoidance_test EXCLUDE_FROM_ALL param_locale_avoidance_test.cpp)

miniros_add_test(${PROJECT_NAME}-crashes_under_gprof EXCLUDE_FROM_ALL crashes_under_gprof.cpp)

miniros_add_test(${PROJECT_NAME}-test_remapping EXCLUDE_FROM_ALL test_remapping.cpp)

miniros_add_test(${PROJECT_NAME}-name_not_remappable EXCLUDE_FROM_ALL name_not_remappable.cpp)

miniros_add_test(${PROJECT_NAME}-test_ns_node_remapping EXCLUDE_FROM_ALL test_ns_node_remapping.cpp)

miniros_add_test(${PROJECT_NAME}-test_search_param EXCLUDE_FROM_ALL test_search_param.cpp)

miniros_add_test(${PROJECT_NAME}-left_right EXCLUDE_FROM_ALL left_right.cpp)
#add_dependencies(${PROJECT_NAME}-left_right ${std_msgs_EXPORTED_TARGETS})

miniros_add_test(${PROJECT_NAME}-string_msg_expect EXCLUDE_FROM_ALL string_msg_expect.cpp)
#add_dependencies(${PROJECT_NAME}-string_msg_expect ${std_msgs_EXPORTED_TARGETS})

miniros_add_test(${PROJECT_NAME}-stamped_topic_statistics_empty_timestamp EXCLUDE_FROM_ALL stamped_topic_statistics_empty_timestamp.cpp)

miniros_add_test(${PROJECT_NAME}-topic_statistic_frequency EXCLUDE_FROM_ALL topic_statistic_frequency.cpp)

# This test requires ability to publish primitive "bool" type, which does not work now.
# But I do not see a big reason to support it.
#miniros_add_test(${PROJECT_NAME}-multiple_latched_publishers EXCLUDE_FROM_ALL multiple_latched_publishers.cpp)
#add_dependencies(${PROJECT_NAME}-multiple_latched_publishers ${std_msgs_EXPORTED_TARGETS})

# The publishers and subscriber are compiled but not registered as a unit test
# since the test execution requires a network connection which drops packages.
# Call scripts/test_udp_with_dropped_packets.sh to run the test.
miniros_add_test(${PROJECT_NAME}-publisher EXCLUDE_FROM_ALL publisher.cpp)
#add_dependencies(${PROJECT_NAME}-publisher ${std_msgs_EXPORTED_TARGETS})

miniros_add_test(${PROJECT_NAME}-publisher_rate EXCLUDE_FROM_ALL publisher_rate.cpp)
#add_dependencies(${PROJECT_NAME}-publisher_rate ${std_msgs_EXPORTED_TARGETS})

miniros_add_test(${PROJECT_NAME}-subscriber EXCLUDE_FROM_ALL subscriber.cpp)
#add_dependencies(${PROJECT_NAME}-subscriber ${std_msgs_EXPORTED_TARGETS})

if(TARGET tests)
  add_dependencies(tests
    ${PROJECT_NAME}-handles
    ${PROJECT_NAME}-timer_callbacks
    ${PROJECT_NAME}-latching_publisher
    ${PROJECT_NAME}-publish_n_fast
    ${PROJECT_NAME}-subscribe_self
    ${PROJECT_NAME}-pub_sub
    ${PROJECT_NAME}-sub_pub
    ${PROJECT_NAME}-publish_empty
    ${PROJECT_NAME}-publish_onsub
    ${PROJECT_NAME}-subscribe_n_fast
    ${PROJECT_NAME}-subscribe_empty
    ${PROJECT_NAME}-subscribe_resubscribe
    ${PROJECT_NAME}-subscribe_unsubscribe
    ${PROJECT_NAME}-publish_unadvertise
    ${PROJECT_NAME}-subscribe_unsubscribe_repeatedly
    ${PROJECT_NAME}-publish_constantly
    ${PROJECT_NAME}-param_update_test
    ${PROJECT_NAME}-real_time_test
    ${PROJECT_NAME}-sim_time_test
    ${PROJECT_NAME}-service_adv
    ${PROJECT_NAME}-service_adv_unadv
    ${PROJECT_NAME}-service_call
    ${PROJECT_NAME}-service_call_zombie
    ${PROJECT_NAME}-service_deadlock
#    ${PROJECT_NAME}-service_exception
    ${PROJECT_NAME}-service_call_repeatedly
    ${PROJECT_NAME}-multiple_init_fini
    ${PROJECT_NAME}-missing_call_to_shutdown
#    ${PROJECT_NAME}-missing_call_to_shutdown_impl
    ${PROJECT_NAME}-inspection
    ${PROJECT_NAME}-service_adv_multiple
    ${PROJECT_NAME}-service_adv_a
    ${PROJECT_NAME}-service_adv_zombie
    ${PROJECT_NAME}-service_wait_a_adv_b
    ${PROJECT_NAME}-service_call_expect_b
    ${PROJECT_NAME}-name_remapping
    ${PROJECT_NAME}-name_remapping_with_ns
    ${PROJECT_NAME}-namespaces
    ${PROJECT_NAME}-params
    ${PROJECT_NAME}-spinners
    ${PROJECT_NAME}-get_master_information
    ${PROJECT_NAME}-multiple_subscriptions
    ${PROJECT_NAME}-check_master
    ${PROJECT_NAME}-wait_for_message
    ${PROJECT_NAME}-loads_of_publishers
    ${PROJECT_NAME}-incrementing_sequence
    ${PROJECT_NAME}-subscription_callback_types
    ${PROJECT_NAME}-service_callback_types
    ${PROJECT_NAME}-intraprocess_subscriptions
    ${PROJECT_NAME}-nonconst_subscriptions
    ${PROJECT_NAME}-subscribe_retry_tcp
    ${PROJECT_NAME}-subscribe_star
    ${PROJECT_NAME}-publisher_for_star_subscriber
    ${PROJECT_NAME}-parameter_validation
    ${PROJECT_NAME}-param_locale_avoidance_test
    ${PROJECT_NAME}-crashes_under_gprof
    ${PROJECT_NAME}-test_remapping
    ${PROJECT_NAME}-name_not_remappable
    ${PROJECT_NAME}-test_ns_node_remapping
    ${PROJECT_NAME}-test_search_param
    ${PROJECT_NAME}-left_right
    ${PROJECT_NAME}-string_msg_expect
    ${PROJECT_NAME}-publisher
    ${PROJECT_NAME}-subscriber
    ${PROJECT_NAME}-stamped_topic_statistics_empty_timestamp
    ${PROJECT_NAME}-topic_statistic_frequency
#    ${PROJECT_NAME}-multiple_latched_publishers
  )
endif()

#add_dependencies(${PROJECT_NAME}-handles ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-timer_callbacks ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-latching_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-publish_n_fast ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-subscribe_self ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-pub_sub ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-sub_pub ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-publish_empty ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-publish_onsub ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-subscribe_n_fast ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-subscribe_empty ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-subscribe_resubscribe ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-subscribe_unsubscribe ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-publish_unadvertise ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-subscribe_unsubscribe_repeatedly ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-publish_constantly ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-param_update_test ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-real_time_test ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-sim_time_test ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-service_adv ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-service_adv_unadv ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-service_call ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-service_call_zombie ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-service_deadlock ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-service_exception ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-service_call_repeatedly ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-multiple_init_fini ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-missing_call_to_shutdown ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-missing_call_to_shutdown_impl ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-inspection ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-service_adv_multiple ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-service_adv_a ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-service_wait_a_adv_b ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-service_call_expect_b ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-service_adv_zombie ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-name_remapping ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-name_remapping_with_ns ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-namespaces ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-params ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-spinners ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-get_master_information ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-multiple_subscriptions ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-check_master ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-wait_for_message ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-loads_of_publishers ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-incrementing_sequence ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-subscription_callback_types ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-service_callback_types ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-intraprocess_subscriptions ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-nonconst_subscriptions ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-subscribe_retry_tcp ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-subscribe_star ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-publisher_for_star_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-parameter_validation ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-param_locale_avoidance_test ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-crashes_under_gprof ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-test_remapping ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-name_not_remappable ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-test_ns_node_remapping ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-test_search_param ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-left_right ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-string_msg_expect ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-stamped_topic_statistics_empty_timestamp ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-topic_statistic_frequency ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}-multiple_latched_publishers ${${PROJECT_NAME}_EXPORTED_TARGETS})
