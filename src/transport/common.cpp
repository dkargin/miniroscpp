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

#include "miniros/transport/common.h"

#include <cstdlib>
#include <cstdio>
#include <cerrno>
#include <cstring>
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

#include <signal.h>

#include "internal_config.h"

#ifdef MINIROS_USE_LIBSYSTEMD

#include <sys/socket.h>
#include <sys/un.h>

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
#if defined(WIN32)
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
  } socket_addr = {
    .sun.sun_family = AF_UNIX,
  };
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
  if (!socket_path)
    return 0; /* Not set? Nothing to do */

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
  if (fd < 0)
    return -errno;

  if (connect(fd, &socket_addr.sa, offsetof(struct sockaddr_un, sun_path) + path_length) != 0)
    return -errno;

  ssize_t written = write(fd, message, message_length);
  if (written != (ssize_t) message_length)
    return written < 0 ? -errno : -EPROTO;

  return 1; /* Notified! */
}

#endif
/// Sends signal to systemd.
/// More info can be found at:
/// https://www.freedesktop.org/software/systemd/man/latest/sd_notify.html#
Error systemdNotify(const char* status)
{
#ifdef HAVE_LIBSYSTEMD
  if (sd_notify(0, status) != 0) {
    return Error::SystemError;
  }
  return Error::Ok;
#else
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
  return systemdNotify("READY=1");
#endif
  return Error::NotSupported;
}


} // namespace miniros
