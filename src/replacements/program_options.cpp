#include <cassert>
#include <limits>
#include <sstream>
#include <cstring>

#include "replacements/program_options/program_options.h"

namespace program_options {

std::string invalid_syntax::get_template(kind_t kind)
{
    // Initially, store the message in 'const char*' variable,
    // to avoid conversion to string in all cases.
    const char* msg;
    switch(kind)
    {
    case empty_adjacent_parameter:
        msg = "the argument for option '%canonical_option%' should follow immediately after the equal sign";
        break;
    case missing_parameter:
        msg = "the required argument for option '%canonical_option%' is missing";
        break;
    case unrecognized_line:
        msg = "the options configuration file contains an invalid line '%invalid_line%'";
        break;
    // none of the following are currently used:
    case long_not_allowed:
        msg = "the unabbreviated option '%canonical_option%' is not valid";
        break;
    case long_adjacent_not_allowed:
        msg = "the unabbreviated option '%canonical_option%' does not take any arguments";
        break;
    case short_adjacent_not_allowed:
        msg = "the abbreviated option '%canonical_option%' does not take any arguments";
        break;
    case extra_parameter:
        msg = "option '%canonical_option%' does not take any arguments";
        break;
    default:
        msg = "unknown command line syntax error for '%s'";
    }
    return msg;
}

////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
///
positional_options_description::~positional_options_description() {

}

positional_options_description& positional_options_description::add(const char* name, int max_count) {
    assert(max_count != -1 || m_trailing.empty());
    if (max_count == -1)
        m_trailing = name;
    else {
        m_names.resize(m_names.size() + max_count, name);
    }
    return *this;
}

unsigned positional_options_description::max_total_count() const
{
    return m_trailing.empty() ?
                static_cast<unsigned>(m_names.size()) : (std::numeric_limits<unsigned>::max)();
}

const std::string& positional_options_description::name_for_position(unsigned position) const
{
    assert(position < max_total_count());

    if (position < m_names.size())
        return m_names[position];
    else
        return m_trailing;
}


////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////

int variables_map::count(const char* key) const {
    return m_variables.count(key);
}

const variables_map::Value& variables_map::operator[](const char* key) const {
    auto it = m_variables.find(key);
    if (it != m_variables.end())
        return *it->second;
    return m_empty;
}

size_t variables_map::size() const {
    return m_variables.size();
}

} // namespace program_options
