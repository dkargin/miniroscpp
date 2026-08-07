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

---

## 2. Single-process candidates (still `DISABLED`)

Same shape as section 1: one gtest (or one node) process, no peer
nodes. Rostest was only used to start the binary (and sometimes pass
args / set params). These can become standalone CTest entries by
enabling them and inlining args/params — **without** `Launcher`.

### Need small inlining (args, params, or env) before enabling

| Binary | Launch XML | What XML provided |
| --- | --- | --- |
| `advanced-subscribe_self` | `subscribe_self.xml` | `args="100 1.0"` |
| `advanced-multiple_init_fini` | `multiple_init_fini.xml` | `args="15"` |
| `advanced-check_master` | `check_master.xml` / `check_master_false.xml` | `args="yes"` / `args="no"` (two scenarios) |
| `advanced-sim_time_test` | `sim_time_test.xml` | `/use_sim_time=true` |
| `advanced-init_no_sim_time_test` | `init_no_sim_time.xml` | `/use_sim_time=true` + `NoSimTime` init |
| `advanced-stamped_topic_statistics_empty_timestamp` | `stamped_topic_statistics_with_empty_timestamp.xml` | `enable_statistics=true` |
| `advanced-name_remapping` | `name_remapping.xml` | remap args + params |
| `advanced-name_remapping_with_ns` | `name_remapping.xml`, `name_remapping_ROS_NAMESPACE.xml` | remaps + params; latter also `ROS_NAMESPACE` |
| `advanced-name_not_remappable` | `name_not_remappable.xml` | remap of `NAME` + param |
| `advanced-namespaces` | `namespaces.xml` | params + `ROS_NAMESPACE` |
| `advanced-test_remapping` | `no_remappings.xml`, `local_remappings.xml`, `global_remappings.xml` | three param/remap scenarios |
| `advanced-param_locale_avoidance_test` | *(no XML in tree)* | locale edge case; confirm expected setup |

Remapping / namespace cases can stay single-process if remaps are passed
as `argv` to `miniros::init` (or set via API before init). They do not
inherently need peer processes.

---

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
| `service_deadlock` | Built `DISABLED`; hangs under CTest (120s timeout). Intended ~30s stress loop; saw `Invalid XML-RPC response` from master. Binary kept for manual runs; `service_deadlock.xml` retained |
| `missing_call_to_shutdown` | Still calls `system("rosrun test_roscpp …")`; `missing_call_to_shutdown_impl` is not built. Needs rewrite with `Launcher` + absolute paths |
| `service_exception` | Not built (`# No support for Log4xx`) |
| `multiple_latched_publishers` | Commented out; needs primitive `bool` publish support |
| `publisher` / `subscriber` / UDP drop | Built `DISABLED`; run via `scripts/test_udp_with_dropped_packets.sh` (needs packet loss) |
| `test-node.xml` | Depends on Python `test_ros` / `test_node_api.py`; no corresponding C++ binary wiring |
| `crashes_under_gprof` | Non-gtest helper-style binary; `DISABLED` |
| `param_update_test` | Non-gtest; `DISABLED`; no launch XML in tree |

---

## 5. Launch XML inventory vs CMake

- Remaining XMLs under `launch/` describe scenarios that are either
  `DISABLED` single-process (section 2) or multi-process (section 3),
  plus blocked cases in section 4.
- XMLs already dropped for section-1 standalone CTest ports:
  `handles.xml`, `timer_callbacks.xml`, `inspection.xml`, `params.xml`,
  `incrementing_sequence.xml`, `subscription_callback_types.xml`,
  `service_callback_types.xml`, `intraprocess_subscriptions.xml`,
  `get_master_information.xml`, `latching_publisher.xml`,
  `real_time_test.xml`, `service_adv_multiple.xml`, `spinners.xml`,
  `wait_for_message.xml`, `nonconst_subscriptions.xml`,
  `parameter_validation.xml`, `ns_node_remapping.xml`, `search_param.xml`.

---

## 6. Suggested migration order

1. Inline args/params for section 2; prefer setting master params in
   `main` (as `params.cpp` does) over keeping XML.
2. Introduce C++ scenario drivers with `miniros::Launcher` for section 3,
   starting with simple pairs (`service_call`, `pubsub_once`) before
   large graphs (`loads_of_publishers`, `topic_statistic_frequency`).
3. Investigate / rewrite `service_deadlock` and `missing_call_to_shutdown`.
