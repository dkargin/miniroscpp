//
// Created by dkargin on 1/30/26.
//

#ifndef MINIROS_INVOKERS_H
#define MINIROS_INVOKERS_H

#include <exception>
#include <functional>

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

/*
template <class Lock, class Functor, class ... Args>
void invokeUnlocked(Lock& lock, Functor f, Args&& ... args)
{
  ScopedUnlock unlock(lock);
  f(std::forward<Args...>(std::forward<Args>(args)...));
}*/

template <class Lock, class Functor, class ... Args>
auto invokeUnlocked(Lock& lock, Functor f, Args&& ... args)
{
  ScopedUnlock unlock(lock);
  return f(std::forward<Args>(args)...);
}

}
#endif // MINIROS_INVOKERS_H
