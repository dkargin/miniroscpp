# Advanced roscpp tests — rostest migration status

Source: `ros_comm` / libros rostest suite. Launch XMLs live in
`test/roscpp/advanced/launch/`. Binaries are built from
`test/roscpp/advanced/CMakeLists.txt`.

**Goal (future):** replace rostest/XML with C++ launchers based on
`miniros::Launcher` (see `test/roscpp/basic/test_late_master.cpp` and
`test/roscpp/basic/test_launcher.cpp` for patterns).

**Current CI:** Linux CI starts a master with `manage-master`, then runs
`ctest -R advanced`. Win32 skips this subdirectory. Rostest is **not**
used for advanced tests in the default CMake path; remaining
`add_rostest(...)` entries under `test/roscpp/basic/CMakeLists.txt` are
catkin-only reference leftovers.

Legend for CMake:

- `miniros_advanced_test` / `miniros_add_test` without `DISABLED` →
  registered as a CTest target (standalone gtest binary).
- `DISABLED` → binary is still built (unless commented out), but not
  registered with CTest.
- Master for registered advanced tests is provided externally
  (`manage-master` / CI), not by rostest.

---

## 1. Standalone gtest — no rostest / XML needed

These are single-process gtest apps. They init ROS in `main`, need at
most a running master, and do **not** require companion nodes or launch
XML. They are registered with CTest today.

| CTest / binary | Source | Notes |
| --- | --- | --- |
| `advanced-test_master_link_execute` | `test_master_link_execute.cpp` | MiniROS-only; fake `HttpServer` peer, **no** real master |
| `advanced-handles` | `handles.cpp` | Former `handles.xml` removed |
| `advanced-timer_callbacks` | `timer_callbacks.cpp` | Former `timer_callbacks.xml` removed |
| `advanced-inspection` | `inspection.cpp` | Former `inspection.xml` removed |
| `advanced-params` | `params.cpp` | Former `params.xml` removed; setup inlined via `MasterLink::set` in `main` |
| `advanced-incrementing_sequence` | `incrementing_sequence.cpp` | Former `incrementing_sequence.xml` removed |
| `advanced-subscription_callback_types` | `subscription_callback_types.cpp` | Former XML removed |
| `advanced-service_callback_types` | `service_callback_types.cpp` | Former XML removed |
| `advanced-intraprocess_subscriptions` | `intraprocess_subscriptions.cpp` | Former XML removed |
| `advanced-latching_publisher` | `latching_publisher.cpp` | Former `latching_publisher.xml` removed |
| `advanced-real_time_test` | `real_time_test.cpp` | Former `real_time_test.xml` removed |
| `advanced-service_adv_multiple` | `service_adv_multiple.cpp` | Former `service_adv_multiple.xml` removed |
| `advanced-spinners` | `spinners.cpp` | Former `spinners.xml` removed (XML only split gtest filters) |
| `advanced-get_master_information` | `get_master_information.cpp` | Former XML already removed |
| `advanced-wait_for_message` | `wait_for_message.cpp` | Former `wait_for_message.xml` removed |
| `advanced-nonconst_subscriptions` | `nonconst_subscriptions.cpp` | Former `nonconst_subscriptions.xml` removed |
| `advanced-parameter_validation` | `parameter_validation.cpp` | Former `parameter_validation.xml` removed |
| `advanced-test_ns_node_remapping` | `test_ns_node_remapping.cpp` | Former `ns_node_remapping.xml` removed |
| `advanced-test_search_param` | `test_search_param.cpp` | Former `search_param.xml` removed |
| `advanced-service_deadlock` | `service_deadlock.cpp` | Former `service_deadlock.xml` removed; fixed MiniROS deadlock (see below) |
| `advanced-name_remapping` | `name_remapping.cpp` | Former `name_remapping.xml` (name_remapped case); remaps + param inlined |
| `advanced-name_remapping_with_ns` | `name_remapping_with_ns.cpp` | Former `name_remapping*.xml` ns=`a` case; `__ns` + remaps + params inlined |
| `advanced-name_not_remappable` | `name_not_remappable.cpp` | Former `name_not_remappable.xml`; `NAME:=…` must not change node name |
| `advanced-namespaces` | `namespaces.cpp` | Former `namespaces.xml`; `ROS_NAMESPACE` + params inlined |
| `advanced-test_remapping` | `test_remapping.cpp` | GTest child only (not a CTest entry); argv/params driven |
| `advanced-launch_no_remappings` | `launch_no_remappings.cpp` | Former `no_remappings.xml` via `Launcher` |
| `advanced-launch_local_remappings` | `launch_local_remappings.cpp` | Former `local_remappings.xml` via `Launcher` |
| `advanced-launch_global_remappings` | `launch_global_remappings.cpp` | Former `global_remappings.xml` via `Launcher` |
| `advanced-subscribe_self` | `subscribe_self.cpp` | Former `subscribe_self.xml`; args `100 1.0` inlined |
| `advanced-multiple_init_fini` | `multiple_init_fini.cpp` | Former `multiple_init_fini.xml`; 15 iterations inlined |
| `advanced-check_master` | `check_master.cpp` | GTest child only (not a CTest entry); argv `yes`/`no` |
| `advanced-launch_check_master` | `launch_check_master.cpp` | Former `check_master.xml` via `Launcher` |
| `advanced-launch_check_master_false` | `launch_check_master_false.cpp` | Former `check_master_false.xml` via `Launcher` |
| `advanced-sim_time_test` | `sim_time_test.cpp` | Former `sim_time_test.xml`; `/use_sim_time=true` inlined |
| `advanced-init_no_sim_time_test` | `init_no_sim_time_test.cpp` | Former `init_no_sim_time.xml`; `/use_sim_time=true` + `NoSimTime` |
| `advanced-stamped_topic_statistics_empty_timestamp` | `stamped_topic_statistics_empty_timestamp.cpp` | Former XML; `/enable_statistics=true` inlined |
| `advanced-param_locale_avoidance_test` | `param_locale_avoidance_test.cpp` | No XML; skips if `de_DE.utf8` is missing |

ROS1 holds `shutting_down_mutex_` across `master::execute` too, but classic

## 3. Multi-process scenarios — need `miniros::Launcher` (or equivalent)

These launch XMLs start one or more companion `<node>` processes plus a
`<test>` gtest. The gtest alone is not a complete scenario. Migration
target: a C++ driver that starts helpers with `Launcher`, then runs
assertions (either in-process or as the test child).

### Pub/sub pairs

| Launch XML | Nodes / helpers | Test binary | Typical args |
| --- | --- | --- | --- |
| `pubsub_once.xml` | `publish_n_fast` | `subscribe_n_fast` | pub `1 1 1`; sub `tcp 1 1.0` |
| `pubsub_n_fast.xml` | `publish_n_fast` | `subscribe_n_fast` | pub `10000 1 1`; sub `tcp 10000 0.15` |
| `pubsub_n_fast_udp.xml` | `publish_n_fast` | `subscribe_n_fast` | same counts, `udp` |
| `pubsub_n_fast_large_message.xml` | `publish_n_fast` | `subscribe_n_fast` | large payload sizes |
| `pubsub_empty.xml` | `publish_empty` | `subscribe_empty` | `1000` / `1000 1.5` |
| `pub_onsub.xml` | `publish_onsub` | `subscribe_empty` | `10` / `10 1.5` |
| `pubsub_resub_once.xml` | two `publish_n_fast` | `subscribe_resubscribe` | remapped second topic |
| `pubsub_unsub.xml` | `publish_constantly` | `subscribe_unsubscribe` | — |
| `pubsub_unadv.xml` | `subscribe_unsubscribe_repeatedly` | `publish_unadvertise` | — |
| `pingpong.xml` | `pub_sub` | `sub_pub` | `1` / `500 10.0` |
| `pingpong_large.xml` | `pub_sub` | `sub_pub` | `1000` / `100 5.0` |
| `multiple_subscriptions.xml` | `publish_constantly` | `multiple_subscriptions` | — |
| `subscribe_retry_tcp.xml` | `publish_constantly` (+ `ROSCPP_ENABLE_DEBUG`) | `subscribe_retry_tcp` | — |
| `subscribe_star.xml` | `publisher_for_star_subscriber` | `subscribe_star` | — |
| `loads_of_publishers.xml` | 20× `publish_constantly` (100 commented) | `loads_of_publishers` | `args="20"` |
| `left_right.xml` | `left_right` (with remaps) | two `string_msg_expect` tests | expected strings / topics |

Helper binaries (non-gtest): `publish_n_fast`, `publish_empty`,
`publish_onsub`, `publish_constantly`, `pub_sub`,
`subscribe_unsubscribe_repeatedly`, `publisher_for_star_subscriber`,
`left_right`, …

### Service scenarios

| Launch XML | Nodes / helpers | Test binary |
| --- | --- | --- |
| `service_call.xml` | `service_adv` | `service_call` |
| `service_call_unadv.xml` | `service_call_repeatedly` | `service_adv_unadv` |
| `service_call_zombie.xml` | `service_adv_zombie` | `service_call_zombie` |
| `service_multiple_providers.xml` | `service_adv_a`, `service_wait_a_adv_b` | `service_call_expect_b` |

Helpers: `service_adv`, `service_adv_unadv` (gtest), `service_adv_a`,
`service_wait_a_adv_b`, `service_adv_zombie`, `service_call_repeatedly`,
`service_call_expect_b`.

### Statistics / rate (many nodes)

| Launch XML | Setup | Test binary |
| --- | --- | --- |
| `topic_statistic_frequency.xml` | `enable_statistics` params + 4× (`publisher_rate` + `subscriber`) pairs | `topic_statistic_frequency` |

Helpers: `publisher_rate`, `subscriber` (and `publisher` for the
manual UDP-drop script).

---

## 4. Special / blocked / out of scope for now

| Item | Status |
| --- | --- |
| `missing_call_to_shutdown` | Still calls `system("rosrun test_roscpp …")`; `missing_call_to_shutdown_impl` is not built. Needs rewrite with `Launcher` + absolute paths |
| `service_exception` | Not built (`# No support for Log4xx`) |
| `multiple_latched_publishers` | Commented out; needs primitive `bool` publish support |
| `publisher` / `subscriber` / UDP drop | Built `DISABLED`; run via `scripts/test_udp_with_dropped_packets.sh` (needs packet loss) |
| `test-node.xml` | Depends on Python `test_ros` / `test_node_api.py`; no corresponding C++ binary wiring |
| `crashes_under_gprof` | Non-gtest helper-style binary; `DISABLED` |
| `param_update_test` | Non-gtest; `DISABLED`; no launch XML in tree |

---

## 5. Launch XML inventory vs CMake

- Remaining XMLs under `launch/` describe multi-process scenarios
  (section 3) plus blocked cases in section 4.
- XMLs already dropped for section-1 standalone CTest ports:
  `handles.xml`, `timer_callbacks.xml`, `inspection.xml`, `params.xml`,
  `incrementing_sequence.xml`, `subscription_callback_types.xml`,
  `service_callback_types.xml`, `intraprocess_subscriptions.xml`,
  `get_master_information.xml`, `latching_publisher.xml`,
  `real_time_test.xml`, `service_adv_multiple.xml`, `spinners.xml`,
  `wait_for_message.xml`, `nonconst_subscriptions.xml`,
  `parameter_validation.xml`, `ns_node_remapping.xml`, `search_param.xml`,
  `service_deadlock.xml`, `name_remapping.xml`,
  `name_remapping_ROS_NAMESPACE.xml`, `name_not_remappable.xml`,
  `namespaces.xml`, `no_remappings.xml`, `local_remappings.xml`,
  `global_remappings.xml`, `subscribe_self.xml`, `multiple_init_fini.xml`,
  `check_master.xml`, `check_master_false.xml`, `sim_time_test.xml`,
  `init_no_sim_time.xml`, `stamped_topic_statistics_with_empty_timestamp.xml`.

---

## 6. Suggested migration order

1. Introduce C++ scenario drivers with `miniros::Launcher` for section 3,
   starting with simple pairs (`service_call`, `pubsub_once`) before
   large graphs (`loads_of_publishers`, `topic_statistic_frequency`).
2. Rewrite `missing_call_to_shutdown` last (process lifecycle).
