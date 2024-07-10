#include "replacements/program_options/program_options.h"

namespace program_options {

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


std::string untyped_value::name() const
{
    static std::string arg("arg");
    return arg;
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
