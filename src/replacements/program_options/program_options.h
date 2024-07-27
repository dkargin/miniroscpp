// Copyright Vladimir Prus 2004, modified by Dmitry Kargin 2024.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt
// or copy at http://www.boost.org/LICENSE_1_0.txt

#ifndef MINIROS_PROGRAM_OPTIONS_H
#define MINIROS_PROGRAM_OPTIONS_H

#include <any>
#include <string>
#include <map>
#include <memory>
#include <functional>
#include <vector>

#include "values.h"

namespace program_options {

template <class Container>
struct multitoken {
    multitoken() {}
};

class options_description;

/// Helper class to add options with fancy syntax.
class OptionBuilder {
public:
    OptionBuilder(options_description& root) : m_root(root) {}

    /// Adds key and the description.
    OptionBuilder& operator()(const char* key, const char* description);

    /// Adds typed option.
    OptionBuilder& operator()(const char* key, value_base* tag, const char* description);

protected:
    options_description& m_root;
};

/// Base class for an option.
class option_description {
public:
    option_description(const char* names, const value_base* s);

    option_description(const char* names, const value_base* s, const char* description);

    ~option_description();

    /// Extract a sequence of keys.
    std::vector<std::string> extractKeys() const;

    enum match_result { no_match, full_match, approximate_match };

    /** Given 'option', specified in the input source,
        returns 'true' if 'option' specifies *this.
    */
    match_result match(const std::string& option, bool approx,
                       bool long_ignore_case, bool short_ignore_case) const;

    /** Returns the key that should identify the option, in
        particular in the variables_map class.
        The 'option' parameter is the option spelling from the
        input source.
        If option name contains '*', returns 'option'.
        If long name was specified, it's the long name, otherwise
        it's a short name with prepended '-'.
    */
    const std::string& key(const std::string& option) const;


    /** Returns the canonical name for the option description to enable the user to
        recognised a matching option.
        1) For short options ('-', '/'), returns the short name prefixed.
        2) For long options ('--' / '-') returns the first long name prefixed
        3) All other cases, returns the first long name (if present) or the short
           name, unprefixed.
    */
    std::string canonical_display_name(int canonical_option_style = 0) const;

    const std::string& long_name() const;

    const std::pair<const std::string*, std::size_t> long_names() const;

    /// Explanation of this option
    const std::string& description() const;

    /// Semantic of option's value
    std::shared_ptr<const value_base> semantic() const;

    /// Returns the option name, formatted suitably for usage message.
    std::string format_name() const;

    /** Returns the parameter name and properties, formatted suitably for
        usage message. */
    std::string format_parameter() const;

protected:
    option_description& set_names(const char* name);

    /**
     * a one-character "switch" name - with its prefix,
     * so that this is either empty or has length 2 (e.g. "-c"
     */
    std::string m_short_name;

    /**
     *  one or more names by which this option may be specified
     *  on a command-line or in a config file, which are not
     *  a single-letter switch. The names here are _without_
     * any prefix.
     */
    std::vector<std::string> m_long_names;
    std::string m_description;

    /// Unparsed key.
    std::string m_key;

    std::shared_ptr<const value_base> m_value;
};

class options_description {
public:

    static const unsigned m_default_line_length;

    /** Creates the instance. */
    options_description(unsigned line_length = m_default_line_length,
                        unsigned min_description_length = m_default_line_length / 2);
    /** Creates the instance. The 'caption' parameter gives the name of
        this 'options_description' instance. Primarily useful for output.
        The 'description_length' specifies the number of columns that
        should be reserved for the description text; if the option text
        encroaches into this, then the description will start on the next
        line.
    */
    options_description(const std::string& caption,
                        unsigned line_length = m_default_line_length,
                        unsigned min_description_length = m_default_line_length / 2);

    ~options_description();
    /** Adds new variable description. Throws duplicate_variable_error if
        either short or long name matches that of already present one.
    */
    void add(std::shared_ptr<option_description> desc);

    /** Adds a group of option description. This has the same
        effect as adding all option_descriptions in 'desc'
        individually, except that output operator will show
        a separate group.
        Returns *this.
    */
    options_description& add(const options_description& desc);

    /** Find the maximum width of the option column, including options
        in groups. */
    unsigned get_option_column_width() const;

public:
    /** Returns an object of implementation-defined type suitable for adding
        options to options_description. The returned object will
        have overloaded operator() with parameter type matching
        'option_description' constructors. Calling the operator will create
        new option_description instance and add it.
    */
    OptionBuilder add_options();

/*
    const OptionDesc& find(
        const std::string& name,
        bool approx,
        bool long_ignore_case = false,
        bool short_ignore_case = false) const;
*/
    const option_description* find_nothrow(const std::string& name,
        bool approx,
        bool long_ignore_case = false,
        bool short_ignore_case = false) const;

    const std::vector<std::shared_ptr<option_description> >& options() const;

    /** Produces a human readable output of 'desc', listing options,
        their descriptions and allowed parameters. Other options_description
        instances previously passed to add will be output separately. */
    friend std::ostream& operator<<(std::ostream& os, const options_description& desc);

    /** Outputs 'desc' to the specified stream, calling 'f' to output each
        option_description element. */
    void print(std::ostream& os, unsigned width = 0) const;

private:
    typedef std::map<std::string, int>::const_iterator name2index_iterator;
    typedef std::pair<name2index_iterator, name2index_iterator>
        approximation_range;

    //approximation_range find_approximation(const std::string& prefix) const;

    std::string m_caption;
    const unsigned m_line_length;
    const unsigned m_min_description_length;

    // Data organization is chosen because:
    // - there could be two names for one option
    // - option_add_proxy needs to know the last added option
    std::vector<std::shared_ptr<option_description> > m_options;

    // Whether the option comes from one of declared groups.
    std::vector<bool> belong_to_group;
    std::vector<std::shared_ptr<options_description> > groups;
};

class positional_options_description {
public:
    ~positional_options_description();

    /** Species that up to 'max_count' next positional options
        should be given the 'name'. The value of '-1' means 'unlimited'.
        No calls to 'add' can be made after call with 'max_value' equal to
        '-1'.
    */
    positional_options_description& add(const char* key, int position);

    /** Returns the maximum number of positional options that can
        be present. Can return (numeric_limits<unsigned>::max)() to
        indicate unlimited number. */
    unsigned max_total_count() const;

    /** Returns the name that should be associated with positional
        options at 'position'.
        Precondition: position < max_total_count()
    */
    const std::string& name_for_position(unsigned position) const;

protected:
    std::vector<std::string> m_names;
    std::string m_trailing;
};

} // namespace program_options

#include "command_line.h"
#include "variables_map.h"

#endif // MINIROS_PROGRAM_OPTIONS_H
