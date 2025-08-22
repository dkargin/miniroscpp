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

#include <random>
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

#include "internal_config.h"

#ifdef HAVE_GLIBC_BACKTRACE
#include <execinfo.h>  // For backtrace()
#endif

#ifdef MINIROS_USE_LIBSYSTEMD

#ifndef WIN32
#include <sys/socket.h>
#include <sys/un.h>
#endif

#ifdef HAVE_LIBSYSTEMD
#include <systemd/sd-daemon.h>
#endif
#endif



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
}

#ifdef MINIROS_USE_LIBSYSTEMD

struct SmartFd {
  int fd = -1;

  ~SmartFd()
  {
    if (fd >= 0) {
      close(fd);
    }
  }

  operator int() const { return fd;}
};

/// Portable version of systemd notifier.
static int portableSystemdNotify(const char *message) {

  union sockaddr_union {
    struct sockaddr sa;
    struct sockaddr_un sun;
  } socket_addr = {};

  socket_addr.sun.sun_family = AF_UNIX;

  size_t path_length, message_length;
  SmartFd fd;

  const char *socket_path;

  /* Verify the argument first */
  if (!message)
    return -EINVAL;

  message_length = strlen(message);
  if (message_length == 0)
    return -EINVAL;

  /* If the variable is not set, the protocol is a noop */
  socket_path = getenv("NOTIFY_SOCKET");
  if (!socket_path) {
    std::cerr << "No NOTIFY_SOCKET variable was set. No systemd notification will be done." << std::endl;
    return 0; /* Not set? Nothing to do */
  }

  /* Only AF_UNIX is supported, with path or abstract sockets */
  if (socket_path[0] != '/' && socket_path[0] != '@')
    return -EAFNOSUPPORT;

  path_length = strlen(socket_path);
  /* Ensure there is room for NUL byte */
  if (path_length >= sizeof(socket_addr.sun.sun_path))
    return -E2BIG;

  memcpy(socket_addr.sun.sun_path, socket_path, path_length);

  /* Support for abstract socket */
  if (socket_addr.sun.sun_path[0] == '@')
    socket_addr.sun.sun_path[0] = 0;

  fd.fd = socket(AF_UNIX, SOCK_DGRAM|SOCK_CLOEXEC, 0);
  if (fd < 0) {
    std::cerr << "Failed to create sdnotify socket" << std::endl;
    return -errno;
  }

  if (connect(fd, &socket_addr.sa, offsetof(struct sockaddr_un, sun_path) + path_length) != 0) {
    std::cerr << "Failed to connect sdnotify socket" << std::endl;
    return -errno;
  }

  ssize_t written = write(fd, message, message_length);
  if (written != (ssize_t) message_length) {
    std::cerr << "Failed to write sdnotify message" << std::endl;
    return written < 0 ? -errno : -EPROTO;
  }

  return 1; /* Notified! */
}

#endif
/// Sends signal to systemd.
/// More info can be found at:
/// https://www.freedesktop.org/software/systemd/man/latest/sd_notify.html
Error systemdNotify(const char* status)
{
#ifdef HAVE_LIBSYSTEMD
  if (sd_notify(0, status) != 0) {
    return Error::SystemError;
  }
  return Error::Ok;
#elif defined(MINIROS_USE_LIBSYSTEMD)
  if (portableSystemdNotify(status))
    return Error::Ok;
  return Error::SystemError;
#endif
  return Error::NotImplemented;
}

Error notifyNodeStarted()
{
#ifdef MINIROS_USE_LIBSYSTEMD
  return systemdNotify("READY=1");
#endif
  return Error::NotSupported;
}

Error notifyNodeExiting()
{
#ifdef MINIROS_USE_LIBSYSTEMD
  return systemdNotify("STOPPING=1");
#endif
  return Error::NotSupported;
}

static std::random_device              rd;
static std::mt19937                    gen(rd());
static std::uniform_int_distribution<> dis(0, 15);
static std::uniform_int_distribution<> dis2(8, 11);

/// code is taken from https://stackoverflow.com/questions/24365331/how-can-i-generate-uuid-in-c-without-using-boost-library
std::string generatePseudoUuid() {
  std::stringstream ss;
  int i;
  ss << std::hex;
  for (i = 0; i < 8; i++) {
    ss << dis(gen);
  }
  ss << "-";
  for (i = 0; i < 4; i++) {
    ss << dis(gen);
  }
  ss << "-4";
  for (i = 0; i < 3; i++) {
    ss << dis(gen);
  }
  ss << "-";
  ss << dis2(gen);
  for (i = 0; i < 3; i++) {
    ss << dis(gen);
  }
  ss << "-";
  for (i = 0; i < 12; i++) {
    ss << dis(gen);
  };
  return ss.str();
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

#ifdef HAVE_GLIBC_BACKTRACE
static constexpr size_t MAX_STACKTRACE_DEPTH = 64;

static void writeStackTrace(int file_descriptor) {
  void* trace[MAX_STACKTRACE_DEPTH];
  size_t trace_depth = backtrace(trace, MAX_STACKTRACE_DEPTH);
  // Note that we skip the first frame here so this function won't show up in
  // the printed trace.
  backtrace_symbols_fd(trace + 1, trace_depth - 1, file_descriptor);
}

/******************************************************************************/
static void fatalSignalHandler(int signal) {
  // Restore the default signal handler for SIGSEGV in case another one
  // happens, and for the re-issue below.
  std::signal(signal, SIG_DFL);

  // WriteStackTrace should theoretically be safe to call here.
  static constexpr const char error_msg[] =
      "*** fatalSignalHandler stack trace: ***\n";
  // Write using safe `write` function. Skip null character.
  if (write(fileno(stderr), error_msg, sizeof(error_msg) - 1) > 0) {
    writeStackTrace(fileno(stderr));
  }

  // Give dummy function time to run. Use "safe" sleep() function.
  sleep(1);

  // Now that the signal handler has been removed we can simply return. If
  // SIGSEGV/SIGABRT was triggered by an instruction, it will occur again. This
  // time it will be handled by the default handler which triggers a core dump.
  return;
}
#endif

Error handleCrashes()
{
#ifdef HAVE_GLIBC_BACKTRACE
  // Use our function to handle segmentation faults.
  // Could add additional signals like: SIGSEGV, SIGSYS, etc.
  std::signal(SIGSEGV, fatalSignalHandler);
  std::signal(SIGABRT, fatalSignalHandler);
  std::signal(SIGILL, fatalSignalHandler);
#else
  return Error::NotSupported;
#endif
  return Error::Ok;
}


} // namespace miniros
