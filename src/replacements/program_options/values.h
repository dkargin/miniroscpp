// Copyright Vladimir Prus 2004, modified by Dmitry Kargin 2024.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt
// or copy at http://www.boost.org/LICENSE_1_0.txt

#ifndef MINIROS_PROGRAM_OPTIONS_VALUES_H
#define MINIROS_PROGRAM_OPTIONS_VALUES_H

#include <any>
#include <string>
#include <vector>
#include <optional>
#include <limits>

namespace program_options {

/// Base class for typed value.
class value_base {
public:
    virtual ~value_base() {}

    virtual std::string name() const = 0;

    /** The minimum number of tokens for this option that
        should be present on the command line. */
    virtual unsigned min_tokens() const = 0;

    /** The maximum number of tokens for this option that
        should be present on the command line. */
    virtual unsigned max_tokens() const = 0;

    virtual bool is_composing() const = 0;

    /** Returns true if value must be given. Non-optional value
    */
    virtual bool is_required() const = 0;

    /** Parses a group of tokens that specify a value of option.
        Stores the result in 'value_store', using whatever representation
        is desired. May be be called several times if value of the same
        option is specified more than once.
    */

    void parse(std::any& value_store,
               const std::vector<std::string>& new_tokens,
               bool utf8) const;

    virtual void xparse(std::any& value_store, const std::vector<std::string>& new_tokens) const = 0;

    /** Called to assign default value to 'value_store'. Returns
        true if default value is assigned, and false if no default
        value exists. */
    virtual bool apply_default(std::any& value_store) const = 0;

    /** Called when final value of an option is determined.
    */
    //virtual void notify(const std::any& value_store) const = 0;

#ifdef USE_RTTI
    virtual const std::type_info& value_type() const = 0;
#endif
};

inline std::string toString(const std::string& arg) {
    return arg;
}

template <class T>
inline std::string toString(T val) {
    return std::to_string(val);
}

template <class T> class typed_value;

/// Base class for typed value. It implements all the common things,
/// which do not require explicit type.
class base_typed_value : public value_base {
public:
    base_typed_value();
    ~base_typed_value();

    std::string name() const override;

    bool is_required() const override ;

    bool is_composing() const override;

    unsigned min_tokens() const override;

    unsigned max_tokens() const override;

    bool apply_default(std::any& output) const override;

protected:
    std::string m_value_name;

    // Default value is stored as std::any and not
    // as std::optional to avoid unnecessary instantiations.
    std::any m_default_value;
    std::string m_default_value_as_text;
    std::any m_implicit_value;
    std::string m_implicit_value_as_text;
    bool m_composing, m_implicit, m_multitoken, m_zero_tokens, m_required;
    //std::function1<void, const T&> m_notifier;
};

template <class T>
class typed_value : public base_typed_value {
public:
    /** Ctor. The 'store_to' parameter tells where to store
                the value when it's known. The parameter can be NULL. */
    typed_value(T* store_to)
    : m_store_to(store_to)
    {}

    /** Specifies default value, which will be used
        if none is explicitly specified. The type 'T' should
        provide operator<< for ostream.
    */
    typed_value<T>* default_value(const T& v)
    {
        m_default_value = std::any(v);
        m_default_value_as_text = toString(v);
        return this;
    }

    /** Specifies default value, which will be used
        if none is explicitly specified. Unlike the above overload,
        the type 'T' need not provide operator<< for ostream,
        but textual representation of default value must be provided
        by the user.
    */
    typed_value<T>* default_value(const T& v, const std::string& textual)
    {
        m_default_value = std::any(v);
        m_default_value_as_text = textual;
        return this;
    }

    /** Specifies an implicit value, which will be used
        if the option is given, but without an adjacent value.
        Using this implies that an explicit value is optional,
    */
    typed_value<T>* implicit_value(const T &v)
    {
        m_implicit_value = v;
        m_implicit_value_as_text = toString(v);
        return this;
    }

    /** Specifies that the value is composing. See the 'is_composing'
        method for explanation.
    */
    typed_value<T>* composing()
    {
        m_composing = true;
        return this;
    }

    /** Specifies the name used to to the value in help message.  */
    typed_value<T>* value_name(const std::string& name)
    {
        m_value_name = name;
        return this;
    }

    typed_value<T>* multitoken() {
        m_multitoken = true;
        return this;
    }

    typed_value<T>* requred() {
        m_required = true;
        return *this;
    }

    /** If 'value_store' is already initialized, or new_tokens
        has more than one elements, throws. Otherwise, assigns
        the first string from 'new_tokens' to 'value_store', without
        any modifications.
     */
    void xparse(std::any& value_store, const std::vector<std::string>& new_tokens) const;

protected:
    T* m_store_to;
};

template <class T>
typed_value<T>* value() {
    return new typed_value<T>(nullptr);
}

class untyped_value : public value_base  {
public:
    untyped_value(bool zero_tokens = false)
    : m_zero_tokens(zero_tokens)
    {}

    std::string name() const;

    unsigned min_tokens() const;
    unsigned max_tokens() const;

    bool is_composing() const { return false; }

    bool is_required() const { return false; }

    /** If 'value_store' is already initialized, or new_tokens
        has more than one elements, throws. Otherwise, assigns
        the first string from 'new_tokens' to 'value_store', without
        any modifications.
     */
    void xparse(std::any& value_store,
                const std::vector<std::string>& new_tokens) const;

    /** Does nothing. */
    bool apply_default(std::any&) const { return false; }

    /** Does nothing. */
    void notify(const std::any&) const {}

private:
    bool m_zero_tokens;
};

} // namespace program_options

#include "values.inl"

#endif // MINIROS_PROGRAM_OPTIONS_VALUES_H
