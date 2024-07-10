#include "replacements/program_options/program_options.h"

namespace program_options {

namespace validators {
void check_first_occurrence(const std::any& value)
{
    if (value.has_value())
        throw multiple_occurrences();
}
} // namespace validators

/* Validates bool value.
    Any of "1", "true", "yes", "on" will be converted to "1".<br>
    Any of "0", "false", "no", "off" will be converted to "0".<br>
    Case is ignored. The 'xs' vector can either be empty, in which
    case the value is 'true', or can contain explicit value.
*/
void validate(std::any& v, const std::vector<std::string>& xs, bool*, int)
{
    validators::check_first_occurrence(v);
    std::string s(validators::get_single_string(xs, true));

    for (size_t i = 0; i < s.size(); ++i)
        s[i] = char(tolower(s[i]));

    if (s.empty() || s == "on" || s == "yes" || s == "1" || s == "true")
        v = true;
    else if (s == "off" || s == "no" || s == "0" || s == "false")
        v = false;
    else {
        throw invalid_bool_value(s);
    }
}

void validate(std::any& v, const std::vector<std::string>& xs, std::string*, int)
{
    validators::check_first_occurrence(v);
    v = validators::get_single_string(xs);
}

void value_base::parse(std::any& value_store, const std::vector<std::string>& new_tokens, bool utf8) const
{
    if (utf8) {
#ifndef BOOST_NO_STD_WSTRING
        // Need to convert to local encoding. Or do we?
        std::vector<std::string> local_tokens = new_tokens;
        /*
        for (unsigned i = 0; i < new_tokens.size(); ++i) {
            std::wstring w = from_utf8(new_tokens[i]);
            local_tokens.push_back(to_local_8_bit(w));
        }*/
        xparse(value_store, local_tokens);
#else
        throw std::runtime_error("UTF-8 conversion not supported.");
#endif
    } else {
        // Already in local encoding, pass unmodified
        xparse(value_store, new_tokens);
    }
}

////////////////////////////////////////////////////////////////////////////

base_typed_value::base_typed_value()
    : m_composing(false),
    m_implicit(false), m_multitoken(false),
    m_zero_tokens(false), m_required(false)
{}

base_typed_value::~base_typed_value() {}

bool base_typed_value::is_required() const {
    return m_required;
}

bool base_typed_value::is_composing() const { return m_composing; }

unsigned base_typed_value::min_tokens() const
{
    if (m_zero_tokens || m_implicit_value.has_value()) {
        return 0;
    } else {
        return 1;
    }
}

unsigned base_typed_value::max_tokens() const {
    if (m_multitoken) {
        return std::numeric_limits<unsigned>::max();
    } else if (m_zero_tokens) {
        return 0;
    } else {
        return 1;
    }
}

std::string g_arg("arg");

std::string base_typed_value::name() const
{
    std::string const& var = (m_value_name.empty() ? g_arg : m_value_name);
    if (m_implicit_value.has_value() && !m_implicit_value_as_text.empty()) {
        std::string msg = "[=" + var + "(=" + m_implicit_value_as_text + ")]";
        if (m_default_value.has_value() && !m_default_value_as_text.empty())
            msg += " (=" + m_default_value_as_text + ")";
        return msg;
    }
    else if (m_default_value.has_value() && !m_default_value_as_text.empty()) {
        return var + " (=" + m_default_value_as_text + ")";
    } else {
        return var;
    }
}

////////////////////////////////////////////////////////////////////////////

std::string untyped_value::name() const
{
    return g_arg;
}

unsigned untyped_value::min_tokens() const
{
    if (m_zero_tokens)
        return 0;
    else
        return 1;
}

unsigned untyped_value::max_tokens() const
{
    if (m_zero_tokens)
        return 0;
    else
        return 1;
}

void untyped_value::xparse(std::any& value_store,
                      const std::vector<std::string>& new_tokens) const
{
    if (value_store.has_value())
        throw multiple_occurrences();
    if (new_tokens.size() > 1)
        throw multiple_values();
    value_store = new_tokens.empty() ? std::string("") : new_tokens.front();
}

} // namespace program_options
