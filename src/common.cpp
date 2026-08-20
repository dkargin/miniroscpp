/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "miniros/common.h"

#include <cstring>
#include <sstream>
#include <cstdlib>
#include <cstdio>
#include <cerrno>
#include <cstring>
#include <csignal>
#include <filesystem>
#include <cassert>
#include <sys/types.h>

#if defined(WIN32)
#else
#include <unistd.h>
#include <pthread.h>
#endif

#ifdef __linux__
#include <sys/prctl.h>
#endif

#if defined(WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

#include "internal_config.h"

#include <mutex>
#include <thread>
#include <iostream>

#ifdef HAVE_GLIBC_BACKTRACE
#include <execinfo.h>  // For backtrace()
#endif

#ifndef WIN32
#include <sys/socket.h>
#include <sys/un.h>
#include <fcntl.h>
#endif

#ifdef HAVE_LIBSYSTEMD
#include <systemd/sd-daemon.h>
#endif

#include "miniros/transport/rpc_manager.h"
#include "internal/profiling.h"


namespace miniros {

void disableAllSignalsInThisThread()
{
#if !defined(WIN32)
  // pthreads_win32, despite having an implementation of pthread_sigmask,
  // doesn't have an implementation of sigset_t, and also doesn't expose its
  // pthread_sigmask externally.
  sigset_t signal_set;

  /* block all signals */
  sigfillset( &signal_set );
  pthread_sigmask( SIG_BLOCK, &signal_set, NULL );
#endif
}

std::map<std::thread::id, std::string> g_nice_thread_names;
std::mutex g_nice_thread_mutex;

// Following advice at https://stackoverflow.com/questions/10121560/stdthread-naming-your-thread
void setThreadName(const char* threadName) {
#if defined(_WIN32)
  // TODO: Implement.
#elif defined(__linux__)
  prctl(PR_SET_NAME, threadName,0,0,0);
#else
  pthread_t handle = pthread_self();
  pthread_setname_np(handle, threadName);
#endif
  profiling::writeCurrentThreadNameInTrace(threadName);
  std::unique_lock lock(g_nice_thread_mutex);
  g_nice_thread_names[std::this_thread::get_id()] = threadName;
}

std::string getThreadName()
{
  std::unique_lock lock(g_nice_thread_mutex);
  auto id = std::this_thread::get_id();
  if (auto it = g_nice_thread_names.find(id); it != g_nice_thread_names.end()) {
    return it->second;
  }
  std::stringstream ss;
  ss << id;
  return ss.str();
}

namespace {

#if !defined(WIN32)
struct SmartFd {
  int fd = -1;
  ~SmartFd()
  {
    if (fd >= 0)
      close(fd);
  }
  operator int() const { return fd; }
};

/// Send to NOTIFY_SOCKET (sd_notify protocol). Returns 1 notified, 0 no socket, <0 errno-style.
int sendNotifySocket(const char* message)
{
  const char* socket_path = getenv("NOTIFY_SOCKET");
  if (!socket_path)
    return 0;
  if (!message || !*message)
    return -EINVAL;

  const size_t message_length = strlen(message);
  union {
    struct sockaddr sa;
    struct sockaddr_un sun;
  } socket_addr = {};
  socket_addr.sun.sun_family = AF_UNIX;

  if (socket_path[0] != '/' && socket_path[0] != '@')
    return -EAFNOSUPPORT;
  const size_t path_length = strlen(socket_path);
  if (path_length >= sizeof(socket_addr.sun.sun_path))
    return -E2BIG;

  memcpy(socket_addr.sun.sun_path, socket_path, path_length);
  if (socket_addr.sun.sun_path[0] == '@')
    socket_addr.sun.sun_path[0] = 0;

  SmartFd fd;
  fd.fd = socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (fd < 0)
    return -errno;

  if (connect(fd, &socket_addr.sa, offsetof(struct sockaddr_un, sun_path) + path_length) != 0)
    return -errno;

  const ssize_t written = write(fd, message, message_length);
  if (written != static_cast<ssize_t>(message_length))
    return written < 0 ? -errno : -EPROTO;
  return 1;
}
#endif

/// Inherited pipe channel for Launcher (and Windows XP/7 baseline).
Error sendNotifyPipe(const char* message)
{
  if (!message || !*message)
    return Error::InvalidValue;

#if defined(WIN32)
  const char* handleStr = getenv("MINIROS_NOTIFY_HANDLE");
  if (!handleStr || !*handleStr)
    return Error::NotSupported;
  char* end = nullptr;
  const unsigned long long value = _strtoui64(handleStr, &end, 0);
  if (end == handleStr || value == 0)
    return Error::InvalidHandle;
  HANDLE handle = reinterpret_cast<HANDLE>(static_cast<uintptr_t>(value));
  DWORD written = 0;
  const DWORD len = static_cast<DWORD>(strlen(message));
  if (!WriteFile(handle, message, len, &written, nullptr) || written != len)
    return Error::SystemError;
  return Error::Ok;
#else
  const char* fdStr = getenv("MINIROS_NOTIFY_FD");
  if (!fdStr || !*fdStr)
    return Error::NotSupported;
  char* end = nullptr;
  const long fd = strtol(fdStr, &end, 10);
  if (end == fdStr || fd < 0)
    return Error::InvalidHandle;
  const size_t len = strlen(message);
  const ssize_t written = write(static_cast<int>(fd), message, len);
  if (written != static_cast<ssize_t>(len))
    return Error::SystemError;
  return Error::Ok;
#endif
}

Error sendSystemdNotify(const char* message)
{
#ifdef HAVE_LIBSYSTEMD
  // Prefer libsystemd when available; it also honors NOTIFY_SOCKET.
  if (!getenv("NOTIFY_SOCKET"))
    return Error::NotSupported;
  if (sd_notify(0, message) < 0)
    return Error::SystemError;
  return Error::Ok;
#elif !defined(WIN32)
  const int rc = sendNotifySocket(message);
  if (rc > 0)
    return Error::Ok;
  if (rc == 0)
    return Error::NotSupported;
  return Error::SystemError;
#else
  (void)message;
  return Error::NotSupported;
#endif
}

NodeNotifyInfo resolveNotifyInfo(const NodeNotifyInfo& in)
{
  NodeNotifyInfo info = in;
#if defined(WIN32)
  if (info.pid <= 0)
    info.pid = static_cast<int64_t>(GetCurrentProcessId());
#else
  if (info.pid <= 0)
    info.pid = static_cast<int64_t>(getpid());
#endif
  if (info.rpcPort <= 0) {
    const RPCManagerPtr& rpc = RPCManager::instance();
    if (rpc) {
      info.rpcPort = static_cast<int>(rpc->getServerPort());
      if (info.uri.empty() && info.rpcPort > 0)
        info.uri = rpc->getServerUrlStr();
    }
  }
  return info;
}

std::string buildNotifyPayload(const NodeNotifyInfo& info, bool ready)
{
  std::ostringstream oss;
  if (ready)
    oss << "READY=1\n";
  else
    oss << "STOPPING=1\n";
  oss << "MAINPID=" << info.pid << "\n";
  if (info.rpcPort > 0)
    oss << "X_MINIROS_RPC_PORT=" << info.rpcPort << "\n";
  if (!info.uri.empty())
    oss << "X_MINIROS_URI=" << info.uri << "\n";
  return oss.str();
}

Error dispatchNotify(const std::string& payload)
{
  int attempted = 0;
  int succeeded = 0;

  if (getenv("NOTIFY_SOCKET")) {
    ++attempted;
    if (sendSystemdNotify(payload.c_str()))
      ++succeeded;
  }

#if defined(WIN32)
  if (getenv("MINIROS_NOTIFY_HANDLE")) {
#else
  if (getenv("MINIROS_NOTIFY_FD")) {
#endif
    ++attempted;
    if (sendNotifyPipe(payload.c_str()))
      ++succeeded;
  }

  if (attempted == 0)
    return Error::Ok; // Silent no-op when no owner channel is configured.
  if (succeeded > 0)
    return Error::Ok;
  return Error::SystemError;
}

} // namespace

Error notifyNodeStarted()
{
  return notifyNodeStarted(NodeNotifyInfo{});
}

Error notifyNodeStarted(const NodeNotifyInfo& info)
{
  return dispatchNotify(buildNotifyPayload(resolveNotifyInfo(info), true));
}

Error notifyNodeExiting()
{
  return notifyNodeExiting(NodeNotifyInfo{});
}

Error notifyNodeExiting(const NodeNotifyInfo& info)
{
  return dispatchNotify(buildNotifyPayload(resolveNotifyInfo(info), false));
}

Error makeDirectory(const std::string& path)
{
  std::error_code ec;
  if (std::filesystem::exists(path, ec))
    return Error::Ok;
  if (!std::filesystem::create_directories(path, ec)) {
    std::cerr << "Failed to create directory \"" << path << "\" : " << ec.message() <<  std::endl;
    return Error::SystemError;
  }
  return Error::Ok;
}

Error changeCurrentDirectory(const std::string& path)
{
  if (!std::filesystem::exists(path)) {
    std::cerr << "Failed to change current directory \"" << path << "\" - path do not exists" << std::endl;
    return Error::SystemError;
  }
  std::filesystem::current_path(path);
  return Error::Ok;
}

bool isProcessAlive(int pid)
{
  if (pid <= 0)
    return true;

#if defined(WIN32) || defined(_WIN32)
  HANDLE h = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, static_cast<DWORD>(pid));
  if (!h)
    return false;
  DWORD exitCode = 0;
  const BOOL ok = GetExitCodeProcess(h, &exitCode);
  CloseHandle(h);
  return ok && exitCode == STILL_ACTIVE;
#else
  if (::kill(pid, 0) == 0)
    return true;
  return errno == EPERM;
#endif
}

#ifdef HAVE_GLIBC_BACKTRACE
static constexpr size_t MAX_STACKTRACE_DEPTH = 64;

/// Pre-opened crash dump fd (async-signal-safe target). -1 = none.
static int g_crashLogFd = -1;

static void writeStackTrace(int file_descriptor) {
  void* trace[MAX_STACKTRACE_DEPTH];
  size_t trace_depth = backtrace(trace, MAX_STACKTRACE_DEPTH);
  // Note that we skip the first frame here so this function won't show up in
  // the printed trace.
  backtrace_symbols_fd(trace + 1, static_cast<int>(trace_depth) - 1, file_descriptor);
}

static void writeAll(int fd, const char* msg, size_t len)
{
  if (fd < 0 || !msg || len == 0)
    return;
  // Best-effort in a signal handler: retry EINTR, stop on other errors / EOF.
  const int savedErrno = errno;
  const char* p = msg;
  size_t left = len;
  while (left > 0) {
    const ssize_t n = ::write(fd, p, left);
    if (n < 0) {
      if (errno == EINTR)
        continue;
      break;
    }
    if (n == 0)
      break;
    p += static_cast<size_t>(n);
    left -= static_cast<size_t>(n);
  }
  errno = savedErrno;
}

static void fatalSignalHandler(int signal) {
  // Restore the default signal handler for SIGSEGV in case another one
  // happens, and for the re-issue below.
  std::signal(signal, SIG_DFL);

  char header[128];
  // snprintf is not strictly async-signal-safe but widely used in crash handlers;
  // keep it short and stack-local.
  const int headerLen = std::snprintf(header, sizeof(header),
    "*** fatalSignalHandler signal=%d pid=%d stack trace: ***\n", signal, static_cast<int>(::getpid()));

  auto emit = [&](int fd) {
    if (fd < 0)
      return;
    if (headerLen > 0)
      writeAll(fd, header, static_cast<size_t>(headerLen));
    writeStackTrace(fd);
    ::fsync(fd);
  };

  emit(fileno(stderr));
  if (g_crashLogFd >= 0 && g_crashLogFd != fileno(stderr))
    emit(g_crashLogFd);

  // Give I/O a moment, then re-raise so the default handler can dump core / exit.
  sleep(1);
  ::raise(signal);
}

static int openCrashLogFd()
{
  // Prefer an explicit path, then log dirs used by CI / manage-master.
  const char* path = std::getenv("MINIROS_CRASH_LOG");
  char buf[512];
  if (!path || !*path) {
    const char* dir = std::getenv("MINIROS_MASTER_LOG_DIR");
    if (!dir || !*dir)
      dir = std::getenv("ROS_LOG_DIR");
    if (dir && *dir) {
      std::snprintf(buf, sizeof(buf), "%s/miniroscore.crash", dir);
      path = buf;
    } else {
      path = "miniroscore.crash";
    }
  }

  // O_APPEND so concurrent threads don't clobber each other; truncate on open.
  const int fd = ::open(path, O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
  if (fd < 0)
    return -1;
  return fd;
}
#endif

Error handleCrashes()
{
#ifdef HAVE_GLIBC_BACKTRACE
  if (g_crashLogFd < 0)
    g_crashLogFd = openCrashLogFd();

  std::signal(SIGSEGV, fatalSignalHandler);
  std::signal(SIGABRT, fatalSignalHandler);
  std::signal(SIGILL, fatalSignalHandler);
  std::signal(SIGFPE, fatalSignalHandler);
  std::signal(SIGBUS, fatalSignalHandler);
#else
  return Error::NotSupported;
#endif
  return Error::Ok;
}


} // namespace miniros
