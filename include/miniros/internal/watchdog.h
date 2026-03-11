/*
 * Copyright (C) 2025
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

#ifndef MINIROS_INTERNAL_WATCHDOG_H
#define MINIROS_INTERNAL_WATCHDOG_H

#include "miniros/macros.h"
#include "miniros/errors.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

#ifndef WIN32
#include <signal.h>
#include <csignal>
#endif

namespace miniros
{

/**
 * \brief Watchdog timer that raises a signal when timeout expires
 *
 * This class provides a watchdog timer mechanism that will raise a specified
 * signal if the timeout expires. The timeout can be reset by calling watch().
 * This is useful for detecting when a process becomes unresponsive or hangs.
 *
 * The implementation uses a std::thread-based approach for portability across
 * platforms. An internal thread periodically checks if the timeout has expired
 * and raises the signal if needed.
 */
class MINIROS_DECL Watchdog
{
public:
  /**
   * \brief Constructor
   * \param signal Signal number to raise when timeout expires
   *               On POSIX systems, defaults to SIGALRM if not specified
   */
#ifndef WIN32
  explicit Watchdog(int signal = SIGALRM);
#else
  explicit Watchdog(int signal = 0);
#endif

  /**
   * \brief Destructor - disarms the timer and stops the worker thread
   */
  ~Watchdog();

  /**
   * \brief Reset the watchdog timeout
   * \param timeoutMs Timeout in milliseconds. If 0, the timer is disarmed.
   * \return Error::Ok on success, Error::SystemError on failure
   */
  Error watch(int timeoutMs);

  /**
   * \brief Disarm the watchdog timer
   * \return Error::Ok on success, Error::SystemError on failure
   */
  Error disarm();

  /**
   * \brief Check if the watchdog is armed
   * \return true if the timer is currently armed, false otherwise
   */
  bool isArmed() const;

private:
  /**
   * \brief Worker thread function that periodically checks timeout
   */
  void threadFunc();

  int signal_;
  bool armed_;
  std::atomic_bool shutdown_;
  
  std::thread thread_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  
  std::chrono::steady_clock::time_point timeout_time_;
  std::chrono::milliseconds timeout_duration_;
};

} // namespace miniros

#endif // MINIROS_INTERNAL_WATCHDOG_H
