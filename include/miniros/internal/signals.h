#ifndef MINIROS_SIGNALS_H
#define MINIROS_SIGNALS_H

#error "This file is not production ready"

#include <functional>
#include <optional>

#include "observer.h"

namespace miniros {

namespace observer {


template <class T>
class TypedConnection : public Connection {
public:
    TypedConnection() {}

    TypedConnection(impl::Connections* container, const T& payload) : Connection(container), m_payload(payload)
    {}

    ~TypedConnection() {}

protected:
    std::optional<T> m_payload;
};



template <class Signature>
class Signal : public observer::Target {
public:
    template <class ...Args>
    void operator() (Args&&... args) const {
        // TODO: Iterate over all objects.


    }

    using Conn_t = TypedConnection<std::function<Signature>>;
};

} // namespace observer
} // namespace miniros

#endif // MINIROS_SIGNALS_H
