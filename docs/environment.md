# Environment variables #

Here is the list of different envorinment variables, which can influence behaviour of ROS client. This list is relevant to regular roscpp client as well.

**ROSCPP_ENABLE_DEBUG** - exposes the service "~debug/close_all_connections"

**ROSCPP_NO_ROSOUT** - disable rosout log appender.

**ROSOUT_DISABLE_FILE_LOGGING** - forbid any file logging. It can be useful to a lifelong ROS-based services.

**ROS_LOG_DIR** - primary path to ROS log folder.

**ROS_HOME** - secondary path to determine ROS log folder.

**HOME** - the last choice for picking ROS log folder.

**ROSCONSOLE_FORMAT**

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

