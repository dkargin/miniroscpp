// Copyright Vladimir Prus 2004, modified by Dmitry Kargin 2024.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt
// or copy at http://www.boost.org/LICENSE_1_0.txt

#ifndef MINIROS_PROGRAM_OPTIONS_VARIABLES_MAP_H
#define MINIROS_PROGRAM_OPTIONS_VARIABLES_MAP_H

#include <any>
#include <memory>
#include <map>
#include <set>

#include "option.h"

namespace program_options {

class value_base;
template<class charT>
class basic_parsed_options;
class variables_map;

class variable_value {
public:
    variable_value() : m_defaulted(false) {}
    variable_value(const std::any& xv, bool xdefaulted)
    : v(xv), m_defaulted(xdefaulted)
    {}

    /** If stored value if of type T, returns that value. Otherwise,
        throws std::bad_any_cast exception. */
   template<class T>
   const T& as() const {
       return std::any_cast<const T&>(v);
   }
   /** @overload */
   template<class T>
   T& as() {
       return std::any_cast<T&>(v);
   }

    /// Returns true if no value is stored.
    bool empty() const;
    /** Returns true if the value was not explicitly
        given, but has default value. */
    bool defaulted() const;
    /** Returns the contained value. */
    const std::any& value() const;

    /** Returns the contained value. */
    std::any& value();
private:
    std::any v;
    bool m_defaulted;
    // Internal reference to value semantic. We need to run
    // notifications when *final* values of options are known, and
    // they are known only after all sources are stored. By that
    // time options_description for the first source might not
    // be easily accessible, so we need to store semantic here.
    std::shared_ptr<const value_base> m_value_semantic;

    friend void store(const basic_parsed_options<char>& options, variables_map& m, bool);

    friend class variables_map;
};


/** Implements string->string mapping with convenient value casting
    facilities. */
class abstract_variables_map {
public:
    abstract_variables_map();
    abstract_variables_map(const abstract_variables_map* next);

    virtual ~abstract_variables_map() {}

    /** Obtains the value of variable 'name', from *this and
        possibly from the chain of variable maps.

        - if there's no value in *this.
            - if there's next variable map, returns value from it
            - otherwise, returns empty value

        - if there's defaulted value
            - if there's next variable map, which has a non-defaulted
              value, return that
            - otherwise, return value from *this

        - if there's a non-defaulted value, returns it.
    */
    const variable_value& operator[](const std::string& name) const;

    /** Sets next variable map, which will be used to find
       variables not found in *this. */
    void next(abstract_variables_map* next);

private:
    /** Returns value of variable 'name' stored in *this, or
        empty value otherwise. */
    virtual const variable_value& get(const std::string& name) const = 0;

    const abstract_variables_map* m_next;
};

/// It contains parsed values from command line.
class variables_map : public abstract_variables_map, public std::map<std::string, variable_value>
{
public:
    variables_map();
    variables_map(const abstract_variables_map* next);

    // Resolve conflict between inherited operators.
    const variable_value& operator[](const std::string& name) const
    { return abstract_variables_map::operator[](name); }

    // Override to clear some extra fields.
    void clear();

    void notify();

protected:
    /** Implementation of abstract_variables_map::get
                which does 'find' in *this. */
    const variable_value& get(const std::string& name) const;

    /** Names of option with 'final' values \-- which should not
        be changed by subsequence assignments. */
    std::set<std::string> m_final;

    friend void store(const basic_parsed_options<char>& options,
                      variables_map& xm, bool utf8);

    /** Names of required options, filled by parser which has
        access to options_description.
        The map values are the "canonical" names for each corresponding option.
        This is useful in creating diagnostic messages when the option is absent. */
    std::map<std::string, std::string> m_required;
};

class command_line_parser;

void store(const parsed_options& parser, variables_map& vm, bool utf8 = false);

/*
 * Templates/inlines
 */

inline bool variable_value::empty() const
{
    return !v.has_value();
}

inline bool variable_value::defaulted() const
{
    return m_defaulted;
}

inline const std::any& variable_value::value() const
{
    return v;
}

inline std::any& variable_value::value()
{
    return v;
}

} // namespace program_options

#endif // MINIROS_PROGRAM_OPTIONS_VARIABLES_MAP_H
