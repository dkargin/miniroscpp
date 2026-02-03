# MiniROS logging #

All miniros modules are logging to `miniros.` channel. `MINIROS_PACKAGE_NAME` macro is used to specify name of additional channel to log.
By default `MINIROS_PACKAGE_NAME=unknown_package` will be used. Regular ROS was providing this macro from its CMake scripts.
Since miniros minimizes usage of external compile options, this macro should be set manually.

## Setting default log levels ## 

Function `miniros::console::set_logger_level(channel, level)` can be used to set initial log levels.

This example will log to `miniros.listener` channel.

```
#define MINIROS_PACKAGE_NAME listener
MINIROS_DEBUG("Hello logging world");
```

This example will log to `miniros.unknown_package.test1` channel:

```
#undef MINIROS_PACKAGE_NAME
MINIROS_DEBUG_NAMED("test1", "Hello logging world");
```

## Internal logging and debug ##

You can add `__miniros.debug:=1` to remappings to set more verbose internal log.

Notable internal channels:

 - **miniros.http** - root channel for all internal http/XMLRPC components
 - **miniros.http.client** - channel for HTTP client.
 - **miniros.net** - channel for high level network components. It contains classes NetSocket, NetAddress, ... . 
 - **miniros.RPCManager** - channel for **RPCManager**.
 - **miniros.master_link** - everything related to **MasterLink**.
