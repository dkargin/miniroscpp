# ROS Master protocol #

Most of negotiation protocol can be found at:
 - Master API: https://wiki.ros.org/ROS/Master_API
 - Slave(Node) API: https://wiki.ros.org/ROS/Slave_API
 - Parameter API: https://wiki.ros.org/ROS/Parameter%20Server%20API

**MiniROS** follows exactly the same protocol to keep compatibility with original ROS1.
While some additional API calls can be added to **miniroscore** and used by miniros-based nodes, it still must be compatible with any ROS1 client. 


# Internals #

Node is uniquely defined by its name and URI. There should be only one active node with the same name.
If some new node with the same name arrives, then old node must be closed.

# RegistrationManager #

Information about nodes and topics is stored at `RegistrationManager`.
It stores both mapping between topics, services and corresponding NodeRef references, and a collection of NodeRef objects themselves.

Nodes queued in `m_nodesToShutdown` are processed by `Master::update()`: Master may send a Slave API
`shutdown` request if the HTTP connection is still usable, drops the node's registrations, and notifies
remaining subscribers via `publisherUpdate` for topics the node used to publish.

# NodeRef #

It provides both a collection of information about the node, and an interface to interact with this node.

When a node registers publishers/subscribers/services, Master opens an HTTP client to the node's Slave API
and requests `getPid`. The PID is stored for diagnostics only — Master never signals the OS process.

Liveness and cleanup:

1. **On disconnect** — Master attempts to reconnect. If reconnect fails, the node is marked `Dead`.
2. **Periodic probe** — `Master::update()` periodically re-sends `getPid` to verify the Slave API is reachable.
3. **Shutdown queue** — Dead / superseded nodes are placed into `RegistrationManager::m_nodesToShutdown`.
   Processing that queue drops registrations and notifies remaining nodes about updated publications.

The check period is configured with the `--node_check_period` option of `miniroscore` (seconds; `0` disables
periodic checks). Default is 5 seconds.
