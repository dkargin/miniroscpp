#include <cassert>
#include <map>
#include <set>

#include "replacements/program_options/variables_map.h"

#include "replacements/program_options/option.h"
#include "replacements/program_options/program_options.h"

namespace program_options {

// First, performs semantic actions for 'oa'.
// Then, stores in 'm' all options that are defined in 'desc'.
void store(const parsed_options& options, variables_map& xm, bool utf8)
{
    // TODO: what if we have different definition
    // for the same option name during different calls
    // 'store'.
    assert(options.description);
    const options_description& desc = *options.description;

    // We need to access map's operator[], not the overriden version
    // variables_map. Ehmm.. messy.
    std::map<std::string, variable_value>& m = xm;

    std::set<std::string> new_final;

    // Declared once, to please Intel in VC++ mode;
    unsigned i;

    // Declared here so can be used to provide context for exceptions
    std::string option_name;
    std::string original_token;

#ifndef BOOST_NO_EXCEPTIONS
    try
#endif
    {
        // First, convert/store all given options
        for (i = 0; i < options.options.size(); ++i) {
            option_name = options.options[i].string_key;
            // Skip positional options without name
            if (option_name.empty())
                continue;

            // Ignore unregistered option. The 'unregistered'
            // field can be true only if user has explicitly asked
            // to allow unregistered options. We can't store them
            // to variables map (lacking any information about paring),
            // so just ignore them.
            if (options.options[i].unregistered)
                continue;

            // If option has final value, skip this assignment
            if (xm.m_final.count(option_name))
                continue;

            original_token = options.options[i].original_tokens.size() ?
                                    options.options[i].original_tokens[0]     : "";
            const option_description* d = desc.find_nothrow(option_name, false, false, false);

            if (!d) {
                throw unknown_option();
            }

            variable_value& v = m[option_name];
            if (v.defaulted()) {
                // Explicit assignment here erases defaulted value
                v = variable_value();
            }

            d->semantic()->parse(v.value(), options.options[i].value, utf8);

            v.m_value_semantic = d->semantic();

            // The option is not composing, and the value is explicitly
            // provided. Ignore values of this option for subsequent
            // calls to 'store'. We store this to a temporary set,
            // so that several assignment inside *this* 'store' call
            // are allowed.
            if (!d->semantic()->is_composing())
                new_final.insert(option_name);
        }
    }
#ifndef BOOST_NO_EXCEPTIONS
    catch(error_with_option_name& e)
    {
        // add context and rethrow
        e.add_context(option_name, original_token, options.m_options_prefix);
        throw;
    }
#endif
    xm.m_final.insert(new_final.begin(), new_final.end());

    // Second, apply default values and store required options.
    const std::vector<std::shared_ptr<option_description> >& all = desc.options();
    for(i = 0; i < all.size(); ++i)
    {
        const option_description& d = *all[i];
        std::string key = d.key("");
        // FIXME: this logic relies on knowledge of option_description
        // internals.
        // The 'key' is empty if options description contains '*'.
        // In that
        // case, default value makes no sense at all.
        if (key.empty())
        {
            continue;
        }
        if (m.count(key) == 0) {
            std::any def;
            if (d.semantic()->apply_default(def)) {
                m[key] = variable_value(def, true);
                m[key].m_value_semantic = d.semantic();
            }
        }

        // add empty value if this is an required option
        if (d.semantic()->is_required()) {

            // For option names specified in multiple ways, e.g. on the command line,
            // config file etc, the following precedence rules apply:
            //  "--"  >  ("-" or "/")  >  ""
            //  Precedence is set conveniently by a single call to length()
            std::string canonical_name = d.canonical_display_name(options.m_options_prefix);
            if (canonical_name.length() > xm.m_required[key].length())
                xm.m_required[key] = canonical_name;
        }
    }
}

} // namespace program_options

