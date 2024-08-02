#ifndef MINIROS_THREAD_LOCAL_PTR_H
#define MINIROS_THREAD_LOCAL_PTR_H

#include <map>
#include <thread>
#include <mutex>

namespace miniros {

template <class T>
class ThreadLocalPointer {
public:
    using thread_id = std::thread::id;

    ~ThreadLocalPointer() {
        std::scoped_lock<std::recursive_mutex> lock(m_guard);
        m_data.clear();
    }

    T* get() {
        std::scoped_lock<std::recursive_mutex> lock(m_guard);
        thread_id id = std::this_thread::get_id();
        auto it = m_data.find(id);
        if (it == m_data.end())
            return nullptr;
        return it->second.get();
    }

    void reset(T* data) {
        std::scoped_lock<std::recursive_mutex> lock(m_guard);
        thread_id id = std::this_thread::get_id();
        m_data[id].reset(data);
    }

    T* operator-> () {
        return get();
    }

protected:
    std::map<thread_id, std::unique_ptr<T>> m_data;
    mutable std::recursive_mutex m_guard;
};

}
#endif // MINIROS_THREAD_LOCAL_PTR_H
