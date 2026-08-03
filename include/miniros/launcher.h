//
// Created by dkargin on 8/31/25.
//

#ifndef MINIROS_LAUNCHER_H
#define MINIROS_LAUNCHER_H

#include <string>
#include <vector>
#include <filesystem>

#include "miniros/macros.h"
#include "miniros/errors.h"
#include "miniros/duration.h"

namespace miniros {

/// Information reported by a child via notifyNodeStarted() (sd_notify-compatible text).
struct MINIROS_DECL ChildReady {
  int64_t pid = 0;
  int rpcPort = 0;
  std::string uri;
  bool ready = false;
};

/**
 * Spawn and supervise a child process.
 *
 * With FLAG_NOTIFY the parent creates a notify channel and injects env for the child:
 * - Unix: NOTIFY_SOCKET (abstract AF_UNIX datagram, same protocol as systemd)
 * - Windows: MINIROS_NOTIFY_HANDLE (inheritable pipe write handle; XP/7-compatible)
 *
 * The child should call notifyNodeStarted() when ready; the parent uses waitReady()
 * to receive MAINPID / X_MINIROS_RPC_PORT / X_MINIROS_URI.
 *
 * Prefer stop() for a cooperative interrupt and terminate() for a forceful kill.
 * Prefer stopAndWait() when tearing down so a stuck child cannot hang the parent forever.
 *
 * Use redirectOutput() to keep stdout/stderr in a file (useful for CI / FLAG_DETACHED).
 */
class MINIROS_DECL Launcher {
public:
  /// Create empty launcher. It can be start later.
  Launcher();
  /// Create launcher for specific PID.
  explicit Launcher(int64_t pid);

  Launcher(const Launcher&) = delete;
  Launcher& operator=(const Launcher&) = delete;
  Launcher(Launcher&&) = delete;
  Launcher& operator=(Launcher&&) = delete;

  ~Launcher();

  enum Flags {
    FLAG_DETACHED = (1 << 0),
    /// Create notify channel and waitReady()-capable supervision (see class comment).
    FLAG_NOTIFY = (1 << 1),
  };

  /// Start executable at path. Fails with FileNotFound if missing, PermissionDenied if not executable.
  Error start(const std::filesystem::path& path, const std::vector<std::string>& args, int flags = 0);

  /// Add environment variables (merged with the parent environment for the child).
  Launcher& env(const char* name, const char* value);

  /// Capture child stdout and stderr into @p path.
  /// Parent directories are created if needed; the file is truncated when start() runs.
  /// Empty path disables capture (default). Call before start().
  /// Works for both attached and FLAG_DETACHED children (detached otherwise discards stdio).
  Launcher& redirectOutput(const std::filesystem::path& path);

  /// Path last passed to redirectOutput(); empty if unset.
  const std::filesystem::path& outputLog() const;

  /// Cooperative stop: SIGINT on POSIX, Ctrl-Break / console interrupt on Windows.
  Error stop();

  /// Forceful kill: SIGKILL on POSIX, TerminateProcess on Windows.
  Error terminate();

  /// Send a raw OS signal (POSIX) or map common signals on Windows. Prefer stop()/terminate().
  Error signal(int signal);

  /// Block until READY=1 on the notify channel (requires FLAG_NOTIFY).
  Error waitReady(ChildReady* out = nullptr, const WallDuration& timeout = WallDuration(10.0));

  /// Last READY payload (valid after successful waitReady).
  const ChildReady& childReady() const;

  /// Wait for the child to exit; returns process exit status (or 128+signum if killed by signal).
  int waitExit();

  /// Wait for the child to exit up to \p timeout.
  /// On success reaps the child (same as waitExit()) and optionally writes the status to \p exitCode.
  /// Returns Error::Timeout if the child is still running when the deadline hits.
  Error waitExit(const WallDuration& timeout, int* exitCode = nullptr);

  /// Cooperative stop(), wait up to \p grace, then terminate() if still alive, then reap.
  /// Returns the process exit status (same convention as waitExit()), or -1 if nothing was started.
  int stopAndWait(const WallDuration& grace = WallDuration(5.0));

  /// Get PID of a process.
  int64_t pid() const;

  /// True if start() succeeded and the child handle is still tracked.
  bool valid() const;

  /// True if the child process is still running.
  /// If the child has already exited, reaps it (clears the handle) and returns false.
  bool running();

  static int64_t myPid();

protected:
  struct Internal;
  Internal *internal_;
};

} // namespace miniros
#endif // MINIROS_LAUNCHER_H
