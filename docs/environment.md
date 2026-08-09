# Environment variables #

Here is the list of different envorinment variables, which can influence behaviour of ROS client. This list is relevant to regular roscpp client as well.

**ROSCPP_ENABLE_DEBUG** - exposes the service "~debug/close_all_connections"

**ROSCPP_NO_ROSOUT** - disable rosout log appender.

**ROSOUT_DISABLE_FILE_LOGGING** - forbid any file logging. It can be useful to a lifelong ROS-based services.

**ROS_LOG_DIR** - primary path to ROS log folder.

**ROS_HOME** - secondary path to determine ROS log folder.

**HOME** - the last choice for picking ROS log folder.

**MINIROS_MASTER_LOG_DIR** - preferred log directory for CI/`manage-master` (also exported as `ROS_LOG_DIR` to miniroscore). Contains `rosout.log` from the master's Rosout aggregator, plus `miniroscore.console.log` and `miniroscore.crash`.

**MINIROS_MASTER_DEBUG** - if `1`/`true`, `manage-master start` raises `--xmlrpc_log=4`.

**MINIROS_MASTER_CONSOLE_LOG** - path for redirected miniroscore stdout/stderr; set by `manage-master` whenever a log dir is used (detached masters normally discard stdio).

**MINIROS_CRASH_LOG** - file where `handleCrashes()` writes an async-signal-safe stack dump on SIGSEGV/SIGABRT/…; `manage-master` points this at `$MINIROS_MASTER_LOG_DIR/miniroscore.crash`.

Tests that need a live master should call `miniros::test::requireMasterOrExit()` from `test/require_master.h` right after `miniros::init()` (exits **90** on failure). `MasterLink::check()` uses XML-RPC `getPid`. Creating a `NodeHandle` afterwards also registers `/rosout`, which is easy to spot in miniroscore logs. `launch_test.sh` dumps master stacks + rosout/crash/core artifacts when a test exits 90 or the shared master dies mid-run, and prints crash/console excerpts to stderr so they appear in the CI job log. On CI failure, the `Dump-master-crash-logs` workflow step also echoes `bin/late-master-logs/` and `master-logs/` crash/console into the job log (artifact zip still uploaded).

**ROSCONSOLE_FORMAT** - specifies format for logging messages. More details can be found at [logging.md](logging.md)

**ROSCONSOLE_STDOUT_LINE_BUFFERED** - forces buffering of stdout writer. Takes values 0 or 1.

**ROS_HOSTNAME** - uses this value instead of system hostname.

**ROS_IP** - uses this value if present and no ROS_HOSTNAME is specified, else goes to system hostname

# Remappings #

Some parameters can be set through remappings to `miniros::init(...)`

**__name** override for name of node.

**__ns** - namespace of node.

**__ip** - override for IP address.

**__hostname** - override for hostname.

**__tcpros_server_port** - port for TCP transport.

**__rpc_server_port** - port for XMLRPC API.

# Parameters #

Some rosparam settings can also be used to alter behaviour of ROS/MiniROS nodes:

