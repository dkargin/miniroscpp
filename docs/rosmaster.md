# ROS Master protocol #

Most of negotiation protocol can be found at:
 - Master API: https://wiki.ros.org/ROS/Master_API
 - Slave(Node) API: https://wiki.ros.org/ROS/Slave_API
 - Parameter API: https://wiki.ros.org/ROS/Parameter%20Server%20API

**MiniROS** follows exactly the same protocol to keep compatibility with original ROS1.
While some additional API calls can be added to **miniroscore** and used by miniros-based nodes, it still must be compatible with any ROS1 client. 

# HTTP endpoints #

These routes are served on the same port as the XML-RPC Master API. Every path below is **GET**. XML-RPC stays at `/RPC2`.

A graph name keeps its leading `/`, so node `/talker` is `/node//talker` and topic `/chatter` is `/topic//chatter`.

## Status pages

| Path | Response |
|------|----------|
| `GET /` | HTML status: hostname, GUID, uptime, links to `/settings` and `/log`, registered nodes, topics, known hosts, and (when multimaster is enabled) the peer list with pair / disconnect forms. Remote peer masters appear under Discovery, not under Nodes. Node Slave API links use a known IP on the same IPv4 subnet as the browser when one exists, otherwise the first non-loopback address. The link text stays the registered URI. No DNS lookup is done. A link that is still a hostname is shown in red: the browser has no way to test that name, and opening it would require DNS. |
| `GET /settings` | HTML form for local settings: resolve IP, discovery on/off, discovery UDP port, multicast group, token, and persistence. Shows the `cache.<port>` path, and a warning when that file, its directory, or multicast cannot be used. |
| `GET /node/<name>` | HTML page for one node: Slave API URL, flags (`LOCAL`, `FOREIGN`, `MINIROS`, `MASTER`), liveness state, queued requests, subscriptions, publications, and services. A remote peer master also gets a pair form. **404** when the name is unknown. |
| `GET /topic/<name>` | HTML page for one topic: message type, publishers, and subscribers, each linked to its node page. **404** when the topic has no type and no publishers or subscribers. |
| `GET /log` | `rosout.log` as `text/plain`. **404** (`log was not configured`) when file logging is off or the file is missing. |
| `GET /favicon.ico` | Built-in icon. |

The root page links each node and topic into the pages above.

## JSON graph API

`published_topics` and `topic_types` return `application/json` objects of the form `{"/topic": "pkg/Msg", ...}`.

| Path | Contents |
|------|----------|
| `GET /api2/published_topics` | Topics that currently have at least one publisher and a recorded type. Same selection as XML-RPC `getPublishedTopics` with an empty subgraph. |
| `GET /api2/topic_types` | Every topic the master has a type for. The type is stored on the first publisher or subscriber registration and is left unchanged by later registrations. A subscriber-only topic is included here and omitted from `/api2/published_topics`. |
| `GET /api2/settings` | Local settings as JSON: `resolve`, `discovery`, `discovery_port`, `persistence`, plus `persistence_path`, `persistence_exists`, `persistence_readable`, and `persistence_writable`. Query parameters with the same names apply a change (`1`/`0`). Omitted parameters stay as they are. Browsers that submit the form are redirected to `/settings`. **400** for a bad value. **500** if discovery cannot bind the requested port. |
| `GET /api2/node_uri?node=<name>` | JSON array of address strings. Example: `["10.0.0.5","fd00::1"]`. An IPv4 address has no `:`. An IPv6 address contains `:`. Any other form is a later address family. |

`ip` limits that single response to the requested families: `4`, `6`, or a comma-separated list such as `4,6` (`ipv4` / `ipv6` accepted). Omitting `ip` returns every known address. Addresses come from the node's host record, local adapters when the node is on this host, and an IP literal already in the registered URI. The body is empty on error: **400** for a missing `node` or a bad `ip` token, **404** when the node is unknown or no address matches.

## Multimaster

`GET /api2/multimaster`, `.../status`, and `.../help` return JSON: this master's GUID, UDP port, multicast endpoint, whether a token is set, `paired_count`, and `peers[]` (`uuid`, `state`, `uri`, `hostname`, `label`, `address`, `pairable`, token flags, and foreign pub/sub/service counts). `help` also lists `commands`.

| Path | Behavior |
|------|----------|
| `GET /api2/multimaster/connect?uuid=<guid>&token=<secret>` | Pair with a discovered peer. `node=<name>` selects a peer master by graph name instead of `uuid`. |
| `GET /api2/multimaster/disconnect` | Leave the collective (disconnect every paired peer). |

`connect` and `disconnect` answer JSON unless the client sends `Accept: text/html` (a browser form). `format=json` forces JSON. The HTML result redirects back to `/` on success. An unknown command is **404**; a disabled multimaster subsystem is **500**. Pairing rules and status codes are in [multimaster.md](multimaster.md).

## Debug API

Registered only when `miniroscore` is started with `--debugAPI`. Responses are HTML.

| Path | Behavior |
|------|----------|
| `GET /debugAPI` or `GET /debugAPI/help` | Lists available commands. |
| `GET /debugAPI/shutdown` | Requests process exit (`Master::ok()` becomes false). |
| any other command | **404**. |

# Internals #

Node is uniquely defined by its name and URI. There should be only one active node with the same name.
If some new node with the same name arrives, then old node must be closed.
It is still possible for two nodes with the same name to exist, but one of these nodes is expected to close soon.

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

`miniroscore` uses [sd_notify](https://www.freedesktop.org/software/systemd/man/latest/sd_notify.html) (`NOTIFY_SOCKET`) without linking libsystemd. The unit is `scripts/miniroscore.service.in` (`Type=notify`).

Order:

1. **`STATUS=entered main`** — first line of `main()` (also printed to stderr). Proves the binary exec'd; `systemctl status` shows this text while still `activating`.
2. **`READY=1`** — as soon as the XML-RPC/HTTP port is listening. Local nodes may start; do not wait for rosout, multicast join, or `getaddrinfo(hostname)`.
3. **`STATUS=running`** — after rosout / event setup.

The unit is ordered `After=network-pre.target` / `Before=network-online.target` so DHCP and late NICs (VPN, `ham0`) do not delay the master. Multicast join retries via netlink when those interfaces appear.

On start timeout the unit uses `TimeoutStartFailureMode=abort` (`SIGABRT`): `handleCrashes()` writes `$MINIROS_CRASH_LOG` (`/var/log/miniroscore/miniroscore.crash`), then the default handler produces a core (`coredumpctl dump miniroscore`). 

# Multimaster #

See [multimaster.md](multimaster.md) for UDP discovery/pairing and registration sync between
`miniroscore` instances.
