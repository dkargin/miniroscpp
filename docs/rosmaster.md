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
4. **Cache restore** — nodes loaded from disk start in `Restoring`, move to `Recovering` after `getPid`,
   then to `Verified` once `getPublications` / `getSubscriptions` complete.

The check period is configured with the `--node_check_period` option of `miniroscore` (seconds; `0` disables
periodic checks). Default is 5 seconds.

# Persistent state #

`miniroscore` can persist a per-port cache file so a restarted master keeps the same instance GUID
(`/run_id`) and can reattach to nodes that survived the master's downtime.

State is stored as `cache.<port>` in the current working directory (for example
`/var/run/miniroscore/cache.11311` when started with `--dir=/var/run/miniroscore`).
Pass `--no-cache` to disable persistence.

On startup the master:

1. Loads `cache.<port>` once (GUID, nodes, and last-known node states).
2. Reuses that GUID for `/run_id`, or generates a new one on first run.
3. Re-registers each cached node by name + Slave API URI (skipping dead cached states / PIDs).
4. After `getPid` verifies the node, requests `getPublications` / `getSubscriptions` and re-registers them.
5. Restores services from the cache snapshot (ROS Slave API has no `getServices`).
6. Sends `publisherUpdate` as topics are restored.

The cache is rewritten when the graph changes and again on clean shutdown.

# Process readiness #

`miniroscore` uses [sd_notify](https://www.freedesktop.org/software/systemd/man/latest/sd_notify.html) protocol to tell SystemD that master has actually started and ready. Its usage is implied in scripts/miniroscore.service unit.
These notifications do not need any additional external libraries, just the capability of compiler and platform to write into unix socket. 

# Multimaster #

See [multimaster.md](multimaster.md) for UDP discovery/pairing and registration sync between
`miniroscore` instances.
