#include <vector>
#include <map>
#include <set>

#include "replacements/program_options/errors.h"
#include "replacements/program_options/command_line.h"

namespace program_options {

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

invalid_option_value::invalid_option_value(const std::string& bad_value)
: validation_error(validation_error::invalid_option_value)
{
    set_substitute("value", bad_value);
}

invalid_bool_value::invalid_bool_value(const std::string& bad_value)
: validation_error(validation_error::invalid_bool_value)
{
    set_substitute("value", bad_value);
}

error_with_option_name::error_with_option_name( const std::string& template_,
                                              const std::string& option_name,
                                              const std::string& original_token,
                                              int option_style) :
                                    error(template_),
                                    m_option_style(option_style),
                                    m_error_template(template_)
{
    //                     parameter            |     placeholder               |   value
    //                     ---------            |     -----------               |   -----
    set_substitute_default("canonical_option",  "option '%canonical_option%'",  "option");
    set_substitute_default("value",             "argument ('%value%')",         "argument");
    set_substitute_default("prefix",            "%prefix%",                     "");
    m_substitutions["option"] = option_name;
    m_substitutions["original_token"] = original_token;
}

const char* error_with_option_name::what() const throw()
{
    // will substitute tokens each time what is run()
    substitute_placeholders(m_error_template);

    return m_message.c_str();
}

void error_with_option_name::replace_token(const std::string& from, const std::string& to) const
{
    for (;;)
    {
        std::size_t pos = m_message.find(from.c_str(), 0, from.length());
        // not found: all replaced
        if (pos == std::string::npos)
            return;
        m_message.replace(pos, from.length(), to);
    }
}

std::string error_with_option_name::get_canonical_option_prefix() const
{
    switch (m_option_style)
    {
    case command_line_style::allow_dash_for_short:
        return "-";
    case command_line_style::allow_slash_for_short:
        return "/";
    case command_line_style::allow_long_disguise:
        return "-";
    case command_line_style::allow_long:
        return "--";
    case 0:
        return "";
    }
    throw std::logic_error("error_with_option_name::m_option_style can only be "
                           "one of [0, allow_dash_for_short, allow_slash_for_short, "
                           "allow_long_disguise or allow_long]");
}


std::string error_with_option_name::get_canonical_option_name() const
{
    if (!m_substitutions.find("option")->second.length())
        return m_substitutions.find("original_token")->second;

    std::string original_token   = strip_prefixes(m_substitutions.find("original_token")->second);
    std::string option_name      = strip_prefixes(m_substitutions.find("option")->second);

    //  For long options, use option name
    if (m_option_style == command_line_style::allow_long        ||
         m_option_style == command_line_style::allow_long_disguise)
        return get_canonical_option_prefix() + option_name;

    //  For short options use first letter of original_token
    if (m_option_style && original_token.length())
        return get_canonical_option_prefix() + original_token[0];

    // no prefix
    return option_name;
}


void error_with_option_name::substitute_placeholders(const std::string& error_template) const
{
    m_message = error_template;
    std::map<std::string, std::string> substitutions(m_substitutions);
    substitutions["canonical_option"]   = get_canonical_option_name();
    substitutions["prefix"]             = get_canonical_option_prefix();


    //
    //  replace placeholder with defaults if values are missing
    //
    for (std::map<std::string, string_pair>::const_iterator iter = m_substitution_defaults.begin();
          iter != m_substitution_defaults.end(); ++iter)
    {
        // missing parameter: use default
        if (substitutions.count(iter->first) == 0 ||
            substitutions[iter->first].length() == 0)
            replace_token(iter->second.first, iter->second.second);
    }


    //
    //  replace placeholder with values
    //  placeholder are denoted by surrounding '%'
    //
    for (std::map<std::string, std::string>::iterator iter = substitutions.begin();
          iter != substitutions.end(); ++iter)
        replace_token('%' + iter->first + '%', iter->second);
}


void ambiguous_option::substitute_placeholders(const std::string& original_error_template) const
{
    // For short forms, all alternatives must be identical, by
    //      definition, to the specified option, so we don't need to
    //      display alternatives
    if (m_option_style == command_line_style::allow_dash_for_short ||
        m_option_style == command_line_style::allow_slash_for_short)
    {
        error_with_option_name::substitute_placeholders(original_error_template);
        return;
    }


    std::string error_template  = original_error_template;
    // remove duplicates using std::set
    std::set<std::string>   alternatives_set (m_alternatives.begin(), m_alternatives.end());
    std::vector<std::string> alternatives_vec (alternatives_set.begin(), alternatives_set.end());

    error_template += " and matches ";
    // Being very cautious: should be > 1 alternative!
    if (alternatives_vec.size() > 1)
    {
        for (unsigned i = 0; i < alternatives_vec.size() - 1; ++i)
            error_template += "'%prefix%" + alternatives_vec[i] + "', ";
        error_template += "and ";
    }

    // there is a programming error if multiple options have the same name...
    if (m_alternatives.size() > 1 && alternatives_vec.size() == 1)
        error_template += "different versions of ";

    error_template += "'%prefix%" + alternatives_vec.back() + "'";


    // use inherited logic
    error_with_option_name::substitute_placeholders(error_template);
}

std::string validation_error::get_template(kind_t kind)
{
    // Initially, store the message in 'const char*' variable,
    // to avoid conversion to std::string in all cases.
    const char* msg;
    switch(kind)
    {
    case invalid_bool_value:
        msg = "the argument ('%value%') for option '%canonical_option%' is invalid. Valid choices are 'on|off', 'yes|no', '1|0' and 'true|false'";
        break;
    case invalid_option_value:
        msg = "the argument ('%value%') for option '%canonical_option%' is invalid";
        break;
    case multiple_values_not_allowed:
        msg = "option '%canonical_option%' only takes a single argument";
        break;
    case at_least_one_value_required:
        msg = "option '%canonical_option%' requires at least one argument";
        break;
    // currently unused
    case invalid_option:
        msg = "option '%canonical_option%' is not valid";
        break;
    default:
        msg = "unknown error";
    }
    return msg;
}

} // namespace program_options
