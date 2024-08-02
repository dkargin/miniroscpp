#ifndef MINIROS_OBSERVER_H
#define MINIROS_OBSERVER_H

#include <type_traits> // for std::is_base_of

namespace miniros {

namespace observer {

class Connection;

namespace impl {
class Connections;
}

/// Generic connection for observer pattern.
/// The role of connection object:
///   - ensure that connection is severed with the target when owner is destroyed.
///   - ensure that connection is severed if the target is destroyed.
///   - everything above is thread safe.
///
/// All connections to same target are forming intrusive linked list.
class Connection {
public:
    Connection();
    explicit Connection(impl::Connections* connections);

    Connection(const Connection& other) = delete;
    Connection(Connection&& other);
    Connection& operator = (const Connection& other) = delete;
    Connection& operator = (Connection&& other) = delete;

    virtual ~Connection();

    /// Check if this connection is attached to anything.
    bool connected() const {
        return m_container != nullptr;
    }

    /// Disconnect from the target.
    void disconnect();

    Connection* next() {
        return m_next;
    }

    const Connection* next() const {
        return m_next;
    }

    Connection* prev() {
        return m_prev;
    }

    const Connection* prev() const {
        return m_prev;
    }

private:
    impl::Connections* m_container;
    Connection* m_prev;
    Connection* m_next;
    friend class impl::Connections;
};

template <class ConnectionImpl = Connection>
class Iterator {
public:
    Iterator(ConnectionImpl* obj) : m_obj(obj) {}

    // prefix increment
    Iterator& operator++()
    {
        // actual increment takes place here
        if (m_obj)
            m_obj = m_obj->next;
        return *this;
    }

    // postfix increment
    Iterator operator++(int)
    {
        Iterator old(m_obj);
        if (m_obj)
            m_obj = static_cast<ConnectionImpl*>(m_obj->next());
        return old;
    }

    friend bool operator == (const Iterator& a, const Iterator& b) {
        return a.m_obj == b.m_obj;
    }

    friend bool operator != (const Iterator& a, const Iterator& b) {
        return a.m_obj != b.m_obj;
    }

    ConnectionImpl& operator *() {
        return *m_obj;
    }

protected:
    ConnectionImpl* m_obj;
};

/// Observer pattern.
/// Observer attaches to some target object.
/// Target object can emit some events, which are handled by observer.
class TargetBase {
public:
    TargetBase();
    ~TargetBase();

    void lock() const;
    void unlock() const;

protected:
    Connection* objBegin() const;
    Connection* objEnd() const;

    void attach(Connection* connection);

    impl::Connections* m_list = nullptr;
};

template <class ObsImpl>
class Target : public TargetBase {
public:
    static_assert(std::is_base_of<Connection, ObsImpl>::value, "Wrong class parameter");\

    using iterator_t = Iterator<ObsImpl>;

    iterator_t begin() const {
        return iterator_t(static_cast<ObsImpl*>(objBegin()));
    }

    iterator_t end() const {
        return iterator_t(static_cast<ObsImpl*>(objEnd()));
    }

    void attach(ObsImpl* obj) {

    }
};

} // namespace observer
} // namespace miniros

#endif // MINIROS_OBSERVER_H
