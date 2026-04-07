//
// Created by dkargin on 1/30/26.
//

#ifndef MINIROS_SCOPED_LOCKS_H
#define MINIROS_SCOPED_LOCKS_H

#include <cassert>
#include <mutex>

#include "miniros/internal/code_location.h"
#include "miniros/rosconsole/local_log.h"
#include "miniros/rostime.h"

#if defined(__clang__)
#  define THREAD_ANNOTATION_ATTRIBUTE__(x) __attribute__((x))
#else
#  define THREAD_ANNOTATION_ATTRIBUTE__(x)  // no-op
#endif

#define GUARDED_BY(x) THREAD_ANNOTATION_ATTRIBUTE__(guarded_by(x))

#define PT_GUARDED_BY(x) THREAD_ANNOTATION_ATTRIBUTE__(pt_guarded_by(x))

#define CAPABILITY(x) THREAD_ANNOTATION_ATTRIBUTE__(capability(x))

#define SCOPED_CAPABILITY THREAD_ANNOTATION_ATTRIBUTE__(scoped_lockable)

#define REQUIRES(...) THREAD_ANNOTATION_ATTRIBUTE__(requires_capability(__VA_ARGS__))

#define ACQUIRE(...) THREAD_ANNOTATION_ATTRIBUTE__(acquire_capability(__VA_ARGS__))

#define RELEASE(...) THREAD_ANNOTATION_ATTRIBUTE__(release_capability(__VA_ARGS__))


namespace miniros {


/// Helper object to track lifetime of an object and catch situations when callback to the object from external thread
/// can be called in the same moment destructor runs.
/// Template parameter is used to distinguish lifetimes of different types of objects in valgrind/helgrind.
template <class T>
class CAPABILITY("mutex") Lifetime {
  public:
  Lifetime(T* object) : object_(object) {}
  virtual ~Lifetime() {}

  using Lock = std::unique_lock<std::mutex>;

  void lock() const ACQUIRE()
  {
    mutex_.lock();
  }

  void unlock() const RELEASE()
  {
    mutex_.unlock();
  }

  /// Check if internal object is valid.
  bool valid(Lock& /*lock*/) const
  {
    return object_ != nullptr;
  }

  /// Reset pointer to object.
  /// Lifetime becomes invalid.
  void reset()
  {
    Lock lock(mutex_);
    object_ = nullptr;
  }

  Lock getLock() const
  {
    return Lock(mutex_);
  }

  /// Reset pointer to object.
  /// This function assumes that lock is already acquired by getLock().
  void reset(Lock& /*lock*/)
  {
    object_ = nullptr;
  }

  /// Get internal pointer.
  /// Lifetime object must be locked beforehand.
  T* ptr()
  {
    return object_;
  }

  /// Get internal pointer.
  /// Lifetime object must be locked beforehand.
  const T* ptr() const
  {
    return object_;
  }

  protected:
  mutable std::mutex mutex_;
  T* object_ = nullptr;
};

/// Scoped unlock, compatible with standard scoped locks.
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

/// Scoped lock with time tracking capability.
/// It shows a warning if lock acquisition was too long.
template <class Lockable_>
class SCOPED_CAPABILITY TimeCheckLock {
public:
  TimeCheckLock(Lockable_& target, const internal::CodeLocation& loc, double timeout = 0.05) ACQUIRE(target)
    : lockable_(&target), timeout_(timeout), loc_(loc)
  {
    this->lock();
  }

  ~TimeCheckLock() RELEASE()
  {
    if (lockable_ && locked_) {
      lockable_->unlock();
    }
  }

  void lock() ACQUIRE(lockable_)
  {
    if (!lockable_) {
      throw std::runtime_error("No lock");
    }
    lock_start_ = SteadyTime::now();
    lockable_->lock();
    lock_acquired_ = SteadyTime::now();
    locked_ = true;

    auto duration = timeToLock().toSec();
    if (duration > timeout_) {
      LOCAL_WARN("TimeCheckLog long lock at %s for %fs", loc_.str().c_str(), duration);
    }
  }

  void unlock() RELEASE()
  {
    if (!lockable_) {
      throw std::runtime_error("No lock");
    }

    assert(locked_ == true);
    lockable_->unlock();
    locked_ = false;
  }

  /// Get time spent on locking.
  WallDuration timeToLock() const
  {
    return lock_acquired_ - lock_start_;
  }

private:
  Lockable_* lockable_ = nullptr;
  bool locked_ = false;
  internal::CodeLocation loc_;
  double timeout_ = 0;
  SteadyTime lock_start_;
  SteadyTime lock_acquired_;
};

#define LOCK_INIT(Type, lock, timeout) Type(lock, THIS_LOCATION, timeout);


}
#endif // MINIROS_SCOPED_LOCKS_H
