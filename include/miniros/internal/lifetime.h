//
// Created by dkargin on 11/16/25.
//

#ifndef MINIROS_LIFETIME_H
#define MINIROS_LIFETIME_H

#include <mutex>

namespace miniros {

/// Helper object to track lifetime of an object and catch situations when callback to the object from external thread
/// can be called in the same moment destructor runs.
/// Template parameter is used to distinguish lifetimes of different types of objects in valgrind/helgrind.
template <class T>
class Lifetime {
public:
  Lifetime(T* object) : object_(object) {}
  virtual ~Lifetime() {}

  using Lock = std::unique_lock<std::mutex>;

  void lock() const
  {
    mutex_.lock();
  }

  void unlock() const
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

}
#endif // MINIROS_LIFETIME_H
