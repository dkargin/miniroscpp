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
struct Lifetime {
  virtual ~Lifetime() {}
  std::mutex mutex;
  bool alive = true;

  void lock()
  {
    mutex.lock();
  }

  void unlock()
  {
    mutex.unlock();
  }
};

}
#endif // MINIROS_LIFETIME_H
