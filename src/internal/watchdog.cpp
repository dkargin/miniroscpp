//
// Created by dkargin on 1/19/25.
//

#include "miniros/internal/watchdog.h"

#include "rosconsole/local_log.h"
#include "miniros/common.h"

// ROS log will be written to "miniros.watchdog"
#define MINIROS_PACKAGE_NAME "watchdog"

namespace miniros
{

Watchdog::Watchdog(int signal)
  : signal_(signal)
  , armed_(false)
  , shutdown_(false)
{
  thread_ = std::thread(&Watchdog::threadFunc, this);
}

Watchdog::~Watchdog()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    shutdown_ = true;
    armed_ = false;
    condition_.notify_all();
  }

  if (thread_.joinable())
  {
    thread_.join();
  }
}

Error Watchdog::watch(int timeoutMs)
{
  if (timeoutMs <= 0)
  {
    return disarm();
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    timeout_duration_ = std::chrono::milliseconds(timeoutMs);
    timeout_time_ = std::chrono::steady_clock::now() + timeout_duration_;
    armed_ = true;
    condition_.notify_all();
  }
  
  return Error::Ok;
}

Error Watchdog::disarm()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    armed_ = false;
    condition_.notify_all();
  }
  
  return Error::Ok;
}

bool Watchdog::isArmed() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return armed_;
}

void Watchdog::threadFunc()
{
  setThreadName("Watchdog");

  constexpr std::chrono::milliseconds check_interval(50); // Check every 10ms

  while (true)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    
    if (shutdown_)
    {
      break;
    }
    
    if (armed_)
    {
      auto now = std::chrono::steady_clock::now();
      
      if (now >= timeout_time_)
      {
        // Timeout expired - raise the signal
        armed_ = false;
        lock.unlock();
        
#ifndef WIN32
        if (raise(signal_) != 0)
        {
          MINIROS_ERROR_NAMED("watchdog", "Failed to raise signal %d", signal_);
        }
        else
        {
          MINIROS_ERROR_NAMED("watchdog", "Watchdog timeout expired, raised signal %d", signal_);
        }
#else
        MINIROS_WARN_NAMED("watchdog", "Watchdog timeout expired, but signal raising is not supported on Windows");
#endif
        continue;
      }
      
      // Wait until timeout or until notified (for updates)
      auto wait_until = timeout_time_;
      condition_.wait_until(lock, wait_until);
    }
    else
    {
      // Not armed - wait until notified or check periodically
      condition_.wait_for(lock, check_interval);
    }
  }
}

} // namespace miniros
