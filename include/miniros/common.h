/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef MINIROSCPP_COMMON_H
#define MINIROSCPP_COMMON_H

/// This file contains some very common includes.
#include <string>

#include "miniros/console.h"
#include "miniros/rosassert.h"
#include "miniros/internal/forwards.h"
#include "miniros/serialized_message.h"
#include "miniros/errors.h"

namespace miniros
{

MINIROS_DECL void disableAllSignalsInThisThread();

/// Set debug-friendly thread name.
MINIROS_DECL void setThreadName(const char* name);

/// Get debug-friendly thread name.
MINIROS_DECL std::string getThreadName();

/// Payload for readiness / exit notifications (sd_notify-compatible KEY=value lines).
struct MINIROS_DECL NodeNotifyInfo {
  /// Process id. 0 → current process.
  int64_t pid = 0;
  /// XML-RPC / HTTP listen port. 0 → try RPCManager::instance() if started.
  int rpcPort = 0;
  /// Optional URI string (e.g. http://host:port).
  std::string uri;
};

/**
 * Notify owner/systemd that the node is ready.
 *
 * Payload includes READY=1, MAINPID, and optional X_MINIROS_RPC_PORT / X_MINIROS_URI.
 * Delivery channels (any may be set; silent no-op if none):
 * - NOTIFY_SOCKET — systemd Type=notify, or Unix Launcher with FLAG_NOTIFY
 * - MINIROS_NOTIFY_HANDLE — inherited pipe write HANDLE (Windows)
 * - MINIROS_NOTIFY_FD — inherited pipe fd (POSIX; useful in unit tests)
 */
MINIROS_DECL Error notifyNodeStarted();
MINIROS_DECL Error notifyNodeStarted(const NodeNotifyInfo& info);

/// Notify owner/systemd that the node is exiting (STOPPING=1 + MAINPID / optional fields).
MINIROS_DECL Error notifyNodeExiting();
MINIROS_DECL Error notifyNodeExiting(const NodeNotifyInfo& info);

/// UUID has the following hex structure:
/// 8-4-4-4-12
///  0 1 2 3  4 5  6 7  8 9 101112131415
/// xxxxxxxx-xxxx-Mxxx-Nxxx-xxxxxxxxxxxx
/// M and N encode type of UUID. They correspond to character 6 and 8.
struct MINIROS_DECL UUID {
  enum {Dim = 16};
  uint8_t bytes[16] = {};

  void generate();

  /// Reset all values to zero.
  void reset();

  /// Check if UUID is valid.
  bool valid() const;

  /// Canonical string form: 8-4-4-4-12 lowercase hex.
  std::string toString() const;

  /// Parse canonical UUID string (dashes optional). Returns false on failure.
  bool fromString(const std::string& str);
};

MINIROS_DECL bool operator == (const UUID& a, const UUID& b);

MINIROS_DECL Error makeDirectory(const std::string& path);

MINIROS_DECL Error changeCurrentDirectory(const std::string& path);

/// Best-effort check whether a process with the given PID is still alive.
/// @param pid - OS process id; values <= 0 are treated as unknown and return true.
MINIROS_DECL bool isProcessAlive(int pid);

/// Enable printing backtrace during crash (stderr + optional crash log file).
/// Opens `$MINIROS_CRASH_LOG`, or `$MINIROS_MASTER_LOG_DIR/miniroscore.crash`,
/// or `./miniroscore.crash`. For internal / CI usage.
MINIROS_DECL Error handleCrashes();


}

#endif

