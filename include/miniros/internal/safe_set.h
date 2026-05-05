//
// Created by dkargin on 5/5/26.
//

#ifndef MINIROS_SAFE_SET_H
#define MINIROS_SAFE_SET_H

#include <set>
#include <mutex>

/// Thread safe set.
/// All methods are guarded by internal mutex.
template <class _Key, class _Comp =  std::less<_Key>, class _Alloc = std::allocator<_Key>>
class SafeSet : std::set<_Key, _Comp, _Alloc> {
public:
  using parent_t = std::set<_Key, _Comp, _Alloc>;
  using key_type = typename parent_t::key_type;
  using value_type = typename parent_t::value_type;
  using lock_type = std::scoped_lock<std::mutex>;

  ~SafeSet()
  {
    clear();
  }

  void insert(value_type&& value)
  {
    lock_type lock(guard_);
    parent_t::insert(value);
  }

  size_t size() const
  {
    lock_type lock(guard_);
    return parent_t::size();
  }

  void erase(const key_type& key)
  {
    lock_type lock(guard_);
    parent_t::erase(key);
  }

  void clear()
  {
    lock_type lock(guard_);
    parent_t::clear();
  }

protected:
  mutable std::mutex guard_;
};
#endif // MINIROS_SAFE_SET_H
