//
// Created by dkargin on 1/30/26.
//

#ifndef MINIROS_SCOPED_LOCKS_H
#define MINIROS_SCOPED_LOCKS_H

#include <cassert>

#include "miniros/internal/code_location.h"
#include "miniros/rosconsole/local_log.h"
#include "miniros/rostime.h"

namespace miniros {

template <class Lock>
class ScopedUnlock {
public:
  ScopedUnlock(Lock& lock)
    :lock_(lock)
  {
    lock_.unlock();
  }
  ~ScopedUnlock()
  {
    lock_.lock();
  }
private:
  Lock& lock_;
};

template <class Lock> class TimeCheckLock {
public:

  TimeCheckLock(Lock& lock, const internal::CodeLocation& loc, double timeout = 0.05) : lock_(&lock), timeout_(timeout), loc_(loc)
  {
    this->lock();
  }

  ~TimeCheckLock()
  {
    if (lock_ && locked_) {
      lock_->unlock();
    }
  }

  void lock()
  {
    if (lock_) {
      lock_start_ = SteadyTime::now();
      lock_->lock();
      lock_acquired_ = SteadyTime::now();
      locked_ = true;

      auto duration = timeToLock().toSec();
      if (duration > timeout_) {
        LOCAL_WARN("TimeCheckLog long lock at %s for %fs", loc_.str().c_str(), duration);
      }
    }
  }

  void unlock()
  {
    if (lock_) {
      assert(locked_ == true);
      lock_->unlock();
      locked_ = false;
    }
  }

  /// Get time spent on locking.
  WallDuration timeToLock() const
  {
    return lock_acquired_ - lock_start_;
  }

private:
  Lock* lock_ = nullptr;
  bool locked_ = false;
  internal::CodeLocation loc_;
  double timeout_ = 0;
  SteadyTime lock_start_;
  SteadyTime lock_acquired_;
};

#define LOCK_INIT(Type, lock, timeout) Type(lock, THIS_LOCATION, timeout);


}
#endif // MINIROS_SCOPED_LOCKS_H
