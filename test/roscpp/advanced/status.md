# Porting tests #

Analysis has stopped at parameter_validation.xml

## Trivial tests ##

These tests do not take any parameters from roslaunch/rostest, so we can just run their executables with CTest

- get_master_information.xml
- handles.xml
- parameter_validation.xml
- incrementing_sequence.xml
- inspection.xml
- intraprocess_subscriptions.xml
- latching_publisher.xml
- missing_call_to_shutdown.xml
- multiple_latched_publishers.xml
- nonconst_subscriptions.xml
- ns_node_remapping.xml
- parameter_validation.xml
- 

## Complex tests ##

- check_master.xml
- check_master_false.xml
- global_remappings.xml
- init_no_sim_time.xml
- left_right.xml
- loads_of_publishers.xml
- local_remappings.xml - probably can be moved into C++ code.
- multiple_init_fini.xml - probably is trivial but there are some strange parameters supplied.
- multiple_subscriptions.xml
- name_not_remappable.xml
- name_remapping.xml
- name_remapping_ROS_NAMESPACE.xml
- namespaces.xml
- no_remappings.xml
- 

Ported:
 - params.xml - moved all initialization to gtest/main
 - 