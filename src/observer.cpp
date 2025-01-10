#include <cassert>
#include <mutex>

#include <miniros/internal/observer.h>

namespace miniros {

namespace observer {

namespace impl {

/// Thread safe list of connected observers.
/// Both observer and observed are pointing to this object.
class Connections {
public:
    using mutex_t = std::recursive_mutex;

    Connections(TargetBase* owner) {
        resetRoot();
        m_owner = owner;
    }

    ~Connections() {
        // Strongly expecting that owner already issued disconnectAll for all listeners.
        assert(m_owner == nullptr);
        assert(isEmpty());
    }

    void resetRoot() {
        m_root.m_next = &m_root;
        m_root.m_prev = &m_root;
    }

    /// Check if there are any connections.
    bool isEmpty() const {
        std::scoped_lock<mutex_t> lock(m_mutex);
        return m_root.m_next == &m_root;
    }

    /// Check if target object is still attached.
    bool isOrphaned() const {
        std::scoped_lock<mutex_t> lock(m_mutex);
        return m_owner == nullptr;
    }

    void lock() const {
        m_mutex.lock();
    }

    void unlock() const {
        m_mutex.unlock();
    }

    Connection* objBegin() const {
        // Note that head is always a dummy "root" object.
        return m_root.m_next;
    }

    Connection* objEnd() const {
        // Note that head is always a dummy "root" object.
        return &m_root;
    }

    /// Detaches connection from the object.
    /// @returns true if there are no links left and connection list can be safely deleted.
    bool detachConnection(Connection* object, Connection* prev, Connection* next);

    /// Detach owner from connection list
    /// It is often called by owner in its destructor.
    bool detachOwner(TargetBase* owner);

    /// Disconnect all current connection, while keeping link to a owner.
    /// @returns number of disconnected objects.
    size_t disconnectAll();

    void pushBack(Connection* object);

protected:
    TargetBase* m_owner = nullptr;

    /// Special placeholder object to for proper "end" iterator.
    mutable Connection m_root;

    mutable mutex_t m_mutex;

    friend class Connection;
};

bool Connections::detachConnection(Connection *object, Connection *prev, Connection *next) {
    assert(object);
    assert(prev);
    assert(next);
    std::scoped_lock<mutex_t> lock(m_mutex);

    // Since this list is always looped, we do not need to check head and tail directly.
    if (prev) {
        prev->m_next = next;
    }

    if (next) {
        next->m_prev = prev;
    }
    return isEmpty() && isOrphaned();
}

void Connections::pushBack(Connection* object) {
    if (object->m_container == this)
        return;
    if (object->m_container)
        object->disconnect();
    std::scoped_lock<mutex_t> lock(m_mutex);
    // Head and tail always exist due to additional "root" object.
    Connection* tail = m_root.m_prev;
    tail->m_next = object;
    m_root.m_prev = object;

    object->m_prev = tail;
    object->m_next = &m_root;
    object->m_container = this;
}

bool Connections::detachOwner(TargetBase* owner) {
    std::scoped_lock<mutex_t> lock(m_mutex);
    assert(owner == m_owner);
    if (owner == m_owner) {
        this->m_owner = nullptr;
        return isEmpty();
    }
    return false;
}

size_t Connections::disconnectAll() {
    std::scoped_lock<mutex_t> lock(m_mutex);
    Connection* ptr = this->objBegin();
    Connection* end = this->objEnd();
    size_t count = 0;
    while (ptr && ptr != end) {
        auto next = ptr->next();
        ptr->m_prev = nullptr;
        ptr->m_next = nullptr;
        ptr->m_container = nullptr;
        ptr = next;
        count++;
    }
    resetRoot();
    return count;
}

} // namespace impl

TargetBase::TargetBase() {
    m_list = new impl::Connections(this);
}

TargetBase::~TargetBase() {
    assert(m_list);
    if (m_list) {
        m_list->disconnectAll();
        bool detached = m_list->detachOwner(this);
        assert(detached);
        delete m_list;
    }
    m_list = nullptr;
}

size_t TargetBase::disconnectAll() {
    if (m_list) {
        return m_list->disconnectAll();
    }
    return 0;
}

void TargetBase::lock() const {
    if (this->m_list) {
        m_list->lock();
    }
}

void TargetBase::unlock() const {
    if (m_list) {
        m_list->unlock();
    }
}

void TargetBase::attach(Connection* connection) {
    if (m_list) {
        m_list->pushBack(connection);
    }
}

Connection* TargetBase::objBegin() const {
    return m_list ? m_list->objBegin() : nullptr;
}

Connection* TargetBase::objEnd() const {
    return m_list ? m_list->objEnd() : nullptr;
}

bool TargetBase::hasConnections() const {
    return m_list ? !m_list->isEmpty() : false;
}


Connection::Connection() {
    m_container = nullptr;
    m_prev = nullptr;
    m_next = nullptr;
}

Connection::Connection(impl::Connections* container) {
    m_prev = nullptr;
    m_next = nullptr;
    m_container = nullptr;

    if (container) {
        container->pushBack(this);
        m_container = container;
    }
}

Connection::~Connection() {
    disconnect();
}

Connection::Connection(Connection&& other) {
    if (other.m_container) {
        std::scoped_lock<impl::Connections> lock(*other.m_container);

        m_prev = other.m_prev;
        m_next = other.m_next;

        if (m_prev) {
            m_prev->m_next = this;
        }
        if (m_next) {
            m_next->m_prev = this;
        }
        m_container = other.m_container;
    }

    other.m_container = nullptr;
    other.m_next = nullptr;
    other.m_prev = nullptr;
}

bool Connection::connected() const {
  return m_container != nullptr && !m_container->isOrphaned();
}

void Connection::disconnect() {
    if (m_container) {
        if (m_container->detachConnection(this, m_prev, m_next)) {
            delete m_container;
        }
        m_prev = nullptr;
        m_next = nullptr;
        m_container = nullptr;
    }
}

} // namespace observer

} // namespace miniros
