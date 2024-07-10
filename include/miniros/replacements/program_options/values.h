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

template <class T>
class typed_value : public value_base {
public:
    /** Ctor. The 'store_to' parameter tells where to store
                the value when it's known. The parameter can be NULL. */
    typed_value(T* store_to)
    : m_store_to(store_to), m_composing(false),
      m_implicit(false), m_multitoken(false),
      m_zero_tokens(false), m_required(false)
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

    bool is_required() const override {
        return m_required;
    }

    bool apply_default(std::any& output) const {
        if (m_default_value.has_value()) {
            output = m_default_value;
            return true;
        }
        return false;
    }

    std::string name() const override;

    bool is_composing() const { return m_composing; }

    unsigned min_tokens() const
    {
        if (m_zero_tokens || m_implicit_value.has_value()) {
            return 0;
        } else {
            return 1;
        }
    }

    unsigned max_tokens() const {
        if (m_multitoken) {
            return std::numeric_limits<unsigned>::max();
        } else if (m_zero_tokens) {
            return 0;
        } else {
            return 1;
        }
    }

    /** If 'value_store' is already initialized, or new_tokens
        has more than one elements, throws. Otherwise, assigns
        the first string from 'new_tokens' to 'value_store', without
        any modifications.
     */
    void xparse(std::any& value_store, const std::vector<std::string>& new_tokens) const;

protected:
    /*
    std::string m_name;
    bool m_multitoken = false;
    bool m_required = false;
    std::optional<T> m_defaultValue;
    std::string m_defaultValueAsText;*/

    T* m_store_to;

            // Default value is stored as boost::any and not
            // as boost::optional to avoid unnecessary instantiations.
    std::string m_value_name;
    std::any m_default_value;
    std::string m_default_value_as_text;
    std::any m_implicit_value;
    std::string m_implicit_value_as_text;
    bool m_composing, m_implicit, m_multitoken, m_zero_tokens, m_required;
    //boost::function1<void, const T&> m_notifier;
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
