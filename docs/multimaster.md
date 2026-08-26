# MiniROS Multimaster #

Each host runs a local `miniroscore`. ROS1 nodes talk to it via `ROS_MASTER_URI` as usual.
Multiple `miniroscore` instances discover each other over UDP multicast, pair into a
**collective**, then exchange publication / subscription / service registrations so local
nodes can connect directly (TCPROS) to foreign publishers and services.

Parameters are **not** synchronized.

## Encodings

- **`/api2/...`** — JSON is the preferred encoding for HTTP APIs (UI and tooling).
- **`/RPC2/`** — XML-RPC is used almost exclusively here, to stay compatible with other ROS1 clients.
- **Multimaster UDP** — ROS message serialization over UDP for internal inter-master discovery and registration sync.

## Principles

1. One local master per computer; nodes use the standard Master API.
2. A shared LAN may use **one UDP multicast discovery group** across many robots.
3. A **collective** is all masters that share the same **token** on that discovery medium.
   Members form a **full mesh** of pairwise sync links (no hub relay of foreign regs).
4. Same multicast + **different tokens** ⇒ masters see each other as discovered but do
   **not** pair (robots stay isolated on a common LAN).
5. Joining another collective (new token or discovery endpoint) **leaves** the current one
   (BYE + drop foreign registrations) before rejoining.
6. Sync is pairwise **unicast** of **local** pubs/subs/services only; foreign registrations
   are never re-exported; `/rosout` stays local-only; the parameter server is not synced.
7. An operator joins a robot by setting that robot’s token (CLI `--token` or HTTP pair form).

### Example

Robots on one LAN share multicast `239.255.42.42:11312`. Computers `A1`/`B1` use
`--token robot1`; `A2`/`B2` use `--token robot2`. All four discover each other; only
matching tokens pair. An operator sets `--token robot1` to mesh with `A1`/`B1`, later
switches to `robot2` (leave robot1, join robot2).

## Ports and CLI

| Option | Behavior |
|--------|----------|
| (default) | Bind **sync** UDP on the master RPC port; join default multicast discovery group `239.255.42.42:11312`. |
| `--token <secret>` | Collective membership secret. Matching tokens auto-pair. Empty = discovery only (manual pair still allowed between open masters). |
| `--multicast addr:port` | Multicast discovery group (default `239.255.42.42:11312`). Empty / `off` disables multicast. |
| `--discovery <port>` | Override **sync** UDP port (defaults to RPC port). |
| `--peer host:udpPort` | Explicit DISCOVER probe to a peer’s **sync** UDP port (repeatable). Fallback when multicast is blocked. |

If `miniroscore` starts before ethernet is up (typical with systemd on SBCs), multicast join
may fail with `No such device` / `Network is unreachable`. With `MINIROS_USE_NETLINK` (default
on Linux), RTNETLINK notifies when links/addresses appear and discovery rejoins immediately;
otherwise NICs are polled every few seconds. Prefer `After=network-online.target` in the unit
file so the first join usually succeeds.

Ad-hoc Wi‑Fi (IBSS) without a DHCP server is supported for **discovery**: an UP interface is
enough to send/receive DISCOVER/OFFER, even when some peers have no IPv4 yet (packets go out
as interface-scoped UDP, source `0.0.0.0`, like DHCP). Peers still show up on the status page
with hostname/GUID; **pairing and TCPROS need a unicast address** and wait until one appears.

Discovery does **not** require `--token`. Auto-pair requires a shared non-empty token. Manual pairing:
pass `--token` up front, enter the remote collective’s token in the web UI when joining a
tokenized mesh, or leave the field empty to pair two open (no-token) masters.

### Persistence (`cache.<port>`)

When the master cache is enabled (default; disable with `--no-cache`), `cache.<port>` stores
the master's `/run_id` GUID, the **hostname** that wrote the file, registered nodes, and
multimaster pairing entries (`peers[]` with `uuid`, `uri`, `sync_host` / `sync_port`, `state`).
After an unclean restart **on the same host** the GUID is reused so peers can re-pair.
If the file is copied onto another machine (or `ROS_HOSTNAME` changes), the GUID is discarded
and a new identity is generated. If two masters still share a GUID (for example both restored
an old copied cache), they no longer treat each other as self-echo: the remote appears on the
status page as **guid-collision** (red, unpaired, pairing forbidden). Delete `cache.<port>`
on one host or pass `--no-cache` so each master gets a unique identity. The shared `--token`
is still taken from CLI / environment and is not written into the cache file.

### Debug API

| Option | Behavior |
|--------|----------|
| `--debugAPI` | Registers `GET /debugAPI/...` dispatcher. `GET /debugAPI/shutdown` requests master process exit (`Master::ok()` becomes false). Intended for integration tests. |

### Web UI / HTTP API

Root page (`GET /`) shows this master's GUID and lists discovered peer masters with state,
URI, and token flags (HTML for browsers). A peer advertising the same GUID is shown in red
and has no pair button. Machine-readable multimaster control uses JSON under
`/api2/multimaster/...`:

| Path | Behavior |
|------|----------|
| `GET /api2/multimaster` or `.../status` | Pairing status: `guid`, `paired_count`, `peers[]` (`uuid`, `state`, `uri`, `pairable`, …) |
| `GET /api2/multimaster/connect?uuid=...&token=...` | Send pair REQUEST. Token optional for open meshes / when already set; required when joining a remote mesh that has a token. Different token leaves old collective. Rejected with HTTP 409 if the UUID is this master's GUID (cloned identity). |
| `GET /api2/multimaster/disconnect` | Leave collective (BYE all paired peers) |

HTML forms on the root page call the same `connect` / `disconnect` paths.

### Same-host / lab testing

```bash
# Three masters, one collective via multicast + optional peer probes
miniroscore -p 11411 --token demo --multicast 239.255.42.42:11312
miniroscore -p 11511 --token demo --multicast 239.255.42.42:11312
miniroscore -p 11611 --token demo --multicast 239.255.42.42:11312

ROS_MASTER_URI=http://127.0.0.1:11611 ./talker
ROS_MASTER_URI=http://127.0.0.1:11411 ./listener
```

If multicast is unavailable, add `--peer 127.0.0.1:<otherSyncPort>` probes (sync port defaults
to each master’s `-p` value).

## UDP framing

Each datagram is a ROS-serialized `miniros_msgs/MasterUdpHeader` (fixed size, no
TCPROS 4-byte length prefix) followed by the payload bytes. `header.op` selects
the payload message type; remaining bytes after the header are that message
(or empty).

| Field | Type | Description |
|-------|------|-------------|
| `magic` | uint32 | `'M' 'R' 'M' '1'` (first on the wire; non-MM UDP is rejected by peek) |
| `version` | uint8 | Protocol version (`3`) |
| `op` | uint8 | Opcode — selects payload message type |
| `flags` | uint16 | e.g. needs-ack, fragment |
| `checksum` | uint32 | CRC32 of payload (`0` if empty) |
| `seq` | uint32 | Sender sequence |
| `ack_seq` | uint32 | Cumulative ACK |
| `uuid` | uint8[16] | Sender master UUID |
| `token_hash` | uint8[8] | First 8 bytes of SHA-1(token) |
| *(rest)* | bytes | ROS-serialized body for `op` |

`op` constants live on `miniros_msgs/MasterUdpHeader`. Inner payloads use
standard miniros/ROS message serialization (`miniros::serialization`), **without**
the extra 4-byte length prefix used by `serializeMessage`.

### Opcode → message type

| Op | Name | Payload |
|----|------|---------|
| 1 | `Discover` | `miniros_msgs/MasterOffer` (optional; empty still accepted) |
| 2 | `Offer` | `miniros_msgs/MasterOffer` |
| 3 | `Request` | *(empty)* |
| 4 | `Ack` | *(empty)* |
| 5 | `Nak` | `miniros_msgs/MasterNak` |
| 6 | `Bye` | `miniros_msgs/MasterBye` |
| 7 | `Heartbeat` | *(empty)* |
| 10 | `SyncSnapshot` | `miniros_msgs/MasterSync` |
| 11 | `SyncDelta` | `miniros_msgs/MasterSync` |
| 12 | `SyncAck` | *(empty)* |

Message sources live in the standalone `miniros_msgs` package
(`~/workspace/miniros_ws/src/miniros_msgs/msg/`). Generated headers are vendored
under `include/generated/miniros_msgs/`.

Discovery/OFFER may travel on the multicast socket; REQUEST/ACK/sync/heartbeat use **unicast**
to each peer’s sync UDP port (`MasterOffer.host` + `MasterOffer.master_port`).

### Regenerating headers

From the catkin workspace that links `gencxx` and `miniros_msgs`:

```bash
source /opt/ros/one/setup.bash
cd ~/workspace/miniros_ws
export ROS_LANG_DISABLE=gencpp:geneus:genlisp:gennodejs:genpy
cmake --build build --target miniros_msgs_generate_messages_cxx
cp -v devel/include/miniros_msgs/*.hxx \
  ~/workspace/miniroscpp/include/generated/miniros_msgs/
```

## Discovery (DHCP-like)

```
  A (searching)                         B (up, same or other token)
  --------------------                  -------------------------
  DISCOVER (+ MasterOffer) ----------->
       [B lists A from offer / UDP src]
                        <-------------- OFFER (MasterOffer)
       [A lists B]
  [if tokens match]
  REQUEST (unicast sync port) -------->
                        <-------------- ACK
  ... MasterSync / heartbeat (unicast) ...
```

## Pairing policy

- **Automatic** when both masters already have the same non-empty `--token` (full mesh within the collective).
- **Manual** via root-page form:
  - joining a tokenized mesh: enter that collective’s token and press **pair**;
  - open masters (no token on either side): leave the token field empty and press **pair**.
- **Disconnect all** / token or multicast change leaves the collective.
- Incoming REQUEST is accepted when fingerprints match, including both-empty (open mesh).
- Empty token: no auto-pair; discovery still runs; manual open pairing is allowed.

## How registrations are applied

After pair + snapshot/deltas:

1. Remote pubs/subs/services are stored as `NODE_FOREIGN` with real remote URIs.
2. Local subscribers get normal `publisherUpdate` callbacks.
3. Data plane stays peer-to-peer TCPROS.
4. Foreign regs are **not** forwarded to other masters (no transitive relay).

## Out of scope (for now)

- Parameter server sync
- Message / service proxying through the master
- Custom node-name remapping for foreign nodes
- Hub-style re-export of foreign registrations
