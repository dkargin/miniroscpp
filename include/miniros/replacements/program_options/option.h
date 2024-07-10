#ifndef MINIROS_PROGRAM_OPTIONS_OPTION_H
#define MINIROS_PROGRAM_OPTIONS_OPTION_H

#include <string>
#include <vector>

namespace program_options {

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

class options_description;

template<class charT>
class basic_parsed_options {
public:
    explicit basic_parsed_options(const options_description* xdescription, int options_prefix = 0)
        : description(xdescription), m_options_prefix(options_prefix)
    {}
    /** Options found in the source. */
    std::vector< basic_option<charT> > options;
    /** Options description that was used for parsing.
        Parsers should return pointer to the instance of
        option_description passed to them, and issues of lifetime are
        up to the caller. Can be NULL.
     */
    const options_description* description;

    /** Mainly used for the diagnostic messages in exceptions.
     *  The canonical option prefix  for the parser which generated these results,
     *  depending on the settings for basic_command_line_parser::style() or
     *  cmdline::style(). In order of precedence of command_line_style enums:
     *      allow_long
     *      allow_long_disguise
     *      allow_dash_for_short
     *      allow_slash_for_short
    */
    int m_options_prefix;
};

typedef basic_parsed_options<char> parsed_options;
typedef basic_parsed_options<wchar_t> wparsed_options;

} // namespace program_options

#endif // MINIROS_PROGRAM_OPTIONS_OPTION_H
