#include <cassert>
#include <mutex>

#include <miniros/internal/observer.h>

namespace miniros {

namespace observer {

namespace impl {

/// Thread safe list of connected observers.
class Connections {
public:
    using mutex_t = std::recursive_mutex;

    Connections(TargetBase* owner) {
        resetRoot();
        m_owner = owner;
    }

    void resetRoot() {
        m_head = &m_root;
        m_tail = &m_root;
        m_root.m_next = &m_root;
        m_root.m_prev = &m_root;
    }

    /// Check if there are any connections.
    bool isEmpty() const {
        std::scoped_lock<mutex_t> lock(m_mutex);
        return m_head == m_tail;
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
        return m_head->m_next;
    }

    Connection* objEnd() const {
        // Note that head is always a dummy "root" object.
        return m_head;
    }

    /// Detaches connection from the object.
    /// @returns true if there are no links left and connection list can be safely deleted.
    bool detachConnection(Connection* object, Connection* prev, Connection* next);

    /// Detach owner from connection list
    /// It is often called by owner in its destructor.
    bool detachOwner(TargetBase* owner);

    /// Disconnect all current connection, while keeping link to a owner.
    void disconnectAll();

    void pushBack(Connection* object);

protected:
    Connection* m_head = nullptr;
    Connection* m_tail = nullptr;
    TargetBase* m_owner = nullptr;

    /// Special placeholder object to for proper "end" iterator.
    Connection m_root;

    mutable mutex_t m_mutex;

    friend class Connection;
};

bool Connections::detachConnection(Connection *object, Connection *prev, Connection *next) {
    std::scoped_lock<mutex_t> lock(m_mutex);
    if (object == m_head) {
        m_head = m_head->m_next;
    }
    if (object == m_tail) {
        m_tail = m_tail->m_prev;
    }

    if (prev) {
        assert(prev->m_next == object);
        prev->m_next = next;
    }

    if (next) {
        assert(next->m_prev == object);
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
    m_tail->m_next = object;
    m_tail = object;
    object->m_prev = m_tail;
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

void Connections::disconnectAll() {
    std::scoped_lock<mutex_t> lock(m_mutex);
    Connection* ptr = this->objBegin();
    Connection* end = this->objEnd();
    while (ptr && ptr != end) {
        auto next = ptr->next();
        ptr->m_prev = nullptr;
        ptr->m_next = nullptr;
        ptr = next;
    }
    resetRoot();
}

} // namespace impl

TargetBase::TargetBase() {
    m_list = new impl::Connections(this);
}

TargetBase::~TargetBase() {
    if (m_list && m_list->detachOwner(this)) {
        delete m_list;
    }
    m_list = nullptr;
}

void TargetBase::disconnectAll() {
    if (m_list) {
        m_list->disconnectAll();
    }
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
