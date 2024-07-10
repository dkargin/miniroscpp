/// A local replacement of boost::program_options.
/// It should support usage scenarios of miniros/rosbag.
/// No other features are needed.

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
class OptionDesc {
public:
    OptionDesc(const char* names, const value_base* s);

    OptionDesc(const char* names, const value_base* s, const char* description);

    ~OptionDesc();

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
    OptionDesc& set_names(const char* name);

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
    using option_description = OptionDesc;

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
    void add(std::shared_ptr<OptionDesc> desc);

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


/// It contains parsed values from command line.
class variables_map {
public:

    class Value {
    public:

        template <class T> T as() const {
            return std::any_cast<T>(m_data);
        }
    protected:
        std::any m_data;
    };

    int count(const char* key) const;
    const Value& operator[](const char* key) const;
    size_t size() const;

protected:
    std::map<std::string, std::unique_ptr<Value>> m_variables;
    Value m_empty;
};

template <class charT=char>
class basic_option {
public:
    basic_option() : position_key(-1) , unregistered(false) , case_insensitive(false) {}

    basic_option(const std::string& xstring_key, const std::vector< std::string> &xvalue)
    : string_key(xstring_key)
    , position_key(-1)
    , value(xvalue)
    , unregistered(false)
    , case_insensitive(false)
    {}

    /** String key of this option. Intentionally independent of the template
        parameter. */
    std::string string_key;
    /** Position key of this option. All options without an explicit name are
        sequentially numbered starting from 0. If an option has explicit name,
        'position_key' is equal to -1. It is possible that both
        position_key and string_key is specified, in case name is implicitly
        added.
     */
    int position_key;
    /** Option's value */
    std::vector< std::basic_string<charT> > value;
    /** The original unchanged tokens this option was
        created from. */
    std::vector< std::basic_string<charT> > original_tokens;
    /** True if option was not recognized. In that case,
        'string_key' and 'value' are results of purely
        syntactic parsing of source. The original tokens can be
        recovered from the "original_tokens" member.
    */
    bool unregistered;
    /** True if string_key has to be handled
        case insensitive.
    */
    bool case_insensitive;
};

namespace command_line_style {
    /** Various possible styles of options.

    There are "long" options, which start with "--" and "short",
    which start with either "-" or "/". Both kinds can be allowed or
    disallowed, see allow_long and allow_short. The allowed character
    for short options is also configurable.

    Option's value can be specified in the same token as name
    ("--foo=bar"), or in the next token.

    It's possible to introduce long options by the same character as
    short options, see allow_long_disguise.

    Finally, guessing (specifying only prefix of option) and case
    insensitive processing are supported.
    */
    enum style_t {
        /// Allow "--long_name" style
        allow_long = 1,
        /// Allow "-<single character" style
        allow_short = allow_long << 1,
        /// Allow "-" in short options
        allow_dash_for_short = allow_short << 1,
        /// Allow "/" in short options
        allow_slash_for_short = allow_dash_for_short << 1,
        /** Allow option parameter in the same token
            for long option, like in
            @verbatim
            --foo=10
            @endverbatim
        */
        long_allow_adjacent = allow_slash_for_short << 1,
        /** Allow option parameter in the next token for
            long options. */
        long_allow_next = long_allow_adjacent << 1,
        /** Allow option parameter in the same token for
            short options. */
        short_allow_adjacent = long_allow_next << 1,
        /** Allow option parameter in the next token for
            short options. */
        short_allow_next = short_allow_adjacent << 1,
        /** Allow to merge several short options together,
            so that "-s -k" become "-sk". All of the options
            but last should accept no parameter. For example, if
            "-s" accept a parameter, then "k" will be taken as
            parameter, not another short option.
            Dos-style short options cannot be sticky.
        */
        allow_sticky = short_allow_next << 1,
        /** Allow abbreviated spellings for long options,
            if they unambiguously identify long option.
            No long option name should be prefix of other
            long option name if guessing is in effect.
        */
        allow_guessing = allow_sticky << 1,
        /** Ignore the difference in case for long options.
        */
        long_case_insensitive = allow_guessing << 1,
        /** Ignore the difference in case for short options.
        */
        short_case_insensitive = long_case_insensitive << 1,
        /** Ignore the difference in case for all options.
        */
        case_insensitive = (long_case_insensitive | short_case_insensitive),
        /** Allow long options with single option starting character,
            e.g <tt>-foo=10</tt>
        */
        allow_long_disguise = short_case_insensitive << 1,
        /** The more-or-less traditional unix style. */
        unix_style = (allow_short | short_allow_adjacent | short_allow_next
                      | allow_long | long_allow_adjacent | long_allow_next
                      | allow_sticky | allow_guessing
                      | allow_dash_for_short),
        /** The default style. */
        default_style = unix_style
    };
} // namespace command_line_style

class command_line_parser {
public:
    using option = basic_option<char>;

    using style_parser = std::function<std::vector<option> (std::vector<std::string>&)>;

    command_line_parser(int argc, char* const argv[]);
    ~command_line_parser();

    command_line_parser& options(const options_description& desc);
    command_line_parser& positional(const positional_options_description& desc);
    command_line_parser& run();

protected:
    std::vector<option> run_impl();
    void finish_option(option& opt, std::vector<std::string>& other_tokens,
                           const std::vector<style_parser>& style_parsers);

    std::vector<option> parse_long_option(std::vector<std::string>& args);
    std::vector<option> parse_short_option(std::vector<std::string>& args);
    std::vector<option> parse_dos_option(std::vector<std::string>& args);
    std::vector<option> parse_disguised_long_option(std::vector<std::string>& args);
    std::vector<option> parse_terminator(std::vector<std::string>& args);

    //std::vector<option> handle_additional_parser(std::vector<std::string>& args);

    int get_canonical_option_prefix() const;
    bool is_style_active(command_line_style::style_t style) const;

protected:
    /// Raw arguments extracted from command line.
    std::vector<std::string> m_rawArguments;

    const options_description* m_options_description = nullptr;
    const positional_options_description* m_positional_options_description = nullptr;

    command_line_style::style_t m_style;
    bool m_allow_unregistered;
};

void store(const command_line_parser& parser, variables_map& vm);

//////////////////////////////////////////////////////////////////////////////
inline std::string strip_prefixes(const std::string& text)
{
    // "--foo-bar" -> "foo-bar"
    std::string::size_type i = text.find_first_not_of("-/");
    if (i == std::string::npos) {
        return text;
    } else {
        return text.substr(i);
    }
}

} // namespace program_options

#endif // MINIROS_PROGRAM_OPTIONS_H
