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

abstract_variables_map::abstract_variables_map()
: m_next(0)
{}

abstract_variables_map::
abstract_variables_map(const abstract_variables_map* next)
: m_next(next)
{}

const variable_value&
abstract_variables_map::operator[](const std::string& name) const
{
    const variable_value& v = get(name);
    if (v.empty() && m_next)
        return (*m_next)[name];
    else if (v.defaulted() && m_next) {
        const variable_value& v2 = (*m_next)[name];
        if (!v2.empty() && !v2.defaulted())
            return v2;
        else return v;
    } else {
        return v;
    }
}

void abstract_variables_map::next(abstract_variables_map* next)
{
    m_next = next;
}

///////////////////////////////////////////////////////////////////////////////////
variables_map::variables_map()
{}

variables_map::variables_map(const abstract_variables_map* next)
: abstract_variables_map(next)
{}

void variables_map::clear()
{
    std::map<std::string, variable_value>::clear();
    m_final.clear();
    m_required.clear();
}

const variable_value& variables_map::get(const std::string& name) const
{
    static variable_value empty;
    const_iterator i = this->find(name);
    if (i == this->end())
        return empty;
    else
        return i->second;
}

void variables_map::notify()
{
    // This checks if all required options occur
    for (map<std::string, std::string>::const_iterator r = m_required.begin();
         r != m_required.end();
         ++r)
    {
        const std::string& opt = r->first;
        const std::string& display_opt = r->second;
        std::map<std::string, variable_value>::const_iterator iter = find(opt);
        if (iter == end() || iter->second.empty())
        {
            throw required_option(display_opt);
        }
    }

#ifdef SUPPORT_NOTIFY
    // Lastly, run notify actions.
    for (std::map<std::string, variable_value>::iterator k = begin();
         k != end(); ++k)
    {
        /* Users might wish to use variables_map to store their own values
           that are not parsed, and therefore will not have value_semantics
           defined. Do not crash on such values. In multi-module programs,
           one module might add custom values, and the 'notify' function
           will be called after that, so we check that value_sematics is
           not NULL. See:
               https://svn.boost.org/trac/boost/ticket/2782
        */
        if (k->second.m_value_semantic)
            k->second.m_value_semantic->notify(k->second.value());
    }
#endif
}
} // namespace program_options
