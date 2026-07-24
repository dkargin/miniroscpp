#ifndef MINIROS_THREAD_LOCAL_PTR_H
#define MINIROS_THREAD_LOCAL_PTR_H

#include <map>
#include <memory>
#include <mutex>
#include <thread>

namespace miniros {

/// Per-thread pointer stored in a shared map keyed by std::thread::id.
///
/// There is no automatic cleanup when a thread exits. If a thread stores a
/// value via reset(p) and then exits without reset(nullptr), that entry (and
/// the owned T) remains until this ThreadLocalPointer is destroyed. That is a
/// controlled leak acceptable for long-lived objects with a small, fixed set of
/// worker threads (e.g. CallbackQueue). Call reset(nullptr) before thread exit
/// if the slot must be released earlier. A recycled thread id could also
/// observe a previous thread's leftover entry until it is overwritten or erased.
template <class T>
class ThreadLocalPointer {
public:
    using Lock = std::scoped_lock<std::mutex>;
    using thread_id = std::thread::id;

    ~ThreadLocalPointer() {
        Lock lock(m_guard);
        m_data.clear();
    }

    T* get() {
        Lock lock(m_guard);
        thread_id id = std::this_thread::get_id();
        auto it = m_data.find(id);
        if (it == m_data.end())
            return nullptr;
        return it->second.get();
    }

    void reset(T* data) {
        Lock lock(m_guard);
        thread_id id = std::this_thread::get_id();
        if (!data) {
            m_data.erase(id);
            return;
        }
        m_data[id].reset(data);
    }

    T* operator-> () {
        return get();
    }

protected:
    std::map<thread_id, std::unique_ptr<T>> m_data;
    mutable std::mutex m_guard;
};

}
#endif // MINIROS_THREAD_LOCAL_PTR_H
