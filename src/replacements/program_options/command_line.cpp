// Copyright Vladimir Prus 2004, modified by Dmitry Kargin 2024.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt
// or copy at http://www.boost.org/LICENSE_1_0.txt

#include <cassert>
#include <climits>

#include "program_options.h"

namespace program_options {
command_line_parser::command_line_parser(int argc, const char* const argv[]) {
    if (argc > 0) {
        m_executableName = argv[0];
    }

    m_rawArguments.resize(argc-1);
    for (int i = 0; i < argc-1; i++) {
        m_rawArguments[i] = argv[i+1];
    }

    m_style = command_line_style::default_style;
    m_allow_unregistered = false;
}

command_line_parser::~command_line_parser() {

}

command_line_parser& command_line_parser::options(const options_description& desc) {
    m_options_description = &desc;
    return *this;
}

command_line_parser& command_line_parser::positional(const positional_options_description& desc) {
    m_positional_options_description = &desc;
    return *this;
}

parsed_options command_line_parser::run()
{
    // save the canonical prefixes which were used by this cmdline parser
    //    eventually inside the parsed results
    //    This will be handy to format recognisable options
    //    for diagnostic messages if everything blows up much later on
    parsed_options result(m_options_description, get_canonical_option_prefix());
    result.options = run_impl();

    // Presense of parsed_options -> wparsed_options conversion
    // does the trick.
    return result;
}

using option = basic_option<char>;

std::vector<option> command_line_parser::run_impl()
{
    using namespace command_line_style;
    // The parsing is done by having a set of 'style parsers'
    // and trying then in order. Each parser is passed a vector
    // of unparsed tokens and can consume some of them (by
    // removing elements on front) and return a vector of options.
    //
    // We try each style parser in turn, untill some input
    // is consumed. The returned vector of option may contain the
    // result of just syntactic parsing of token, say --foo will
    // be parsed as option with name 'foo', and the style parser
    // is not required to care if that option is defined, and how
    // many tokens the value may take.
    // So, after vector is returned, we validate them.
    assert(m_options_description);

    std::vector<style_parser> style_parsers;

    //if (m_style_parser)
    //    style_parsers.push_back(m_style_parser);

    //if (m_additional_parser) {
    //    style_parsers.push_back([this] (std::vector<std::string>& args) {
    //        return this->handle_additional_parser(args);
    //    });
    //}

    if (m_style & allow_long) {
        style_parsers.push_back(
                    [this] (std::vector<std::string>& args) {
            return this->parse_long_option(args);
        });
    }

    if ((m_style & allow_long_disguise)) {
        style_parsers.push_back(
                    [this] (std::vector<std::string>& args) {
            return this->parse_disguised_long_option(args);
        });
    }

    if ((m_style & allow_short) && (m_style & allow_dash_for_short)) {
        style_parsers.push_back(
                    [this] (std::vector<std::string>& args) {
            return this->parse_short_option(args);
        });
    }

    if ((m_style & allow_short) && (m_style & allow_slash_for_short)) {
        style_parsers.push_back(
                    [this] (std::vector<std::string>& args) {
            return this->parse_dos_option(args);
        });
    }

    style_parsers.push_back(
                [this](std::vector<std::string>& args) {
        return parse_terminator(args);
    });

    std::vector<option> result;
    std::vector<std::string>& args = m_rawArguments;

    while(!args.empty())
    {
        bool ok = false;
        for(unsigned i = 0; i < style_parsers.size(); ++i)
        {
            unsigned current_size = static_cast<unsigned>(args.size());
            std::vector<option> next = style_parsers[i](args);

            // Check that option names
            // are valid, and that all values are in place.
            if (!next.empty())
            {
                std::vector<std::string> e;
                for(unsigned k = 0; k < next.size()-1; ++k) {
                    finish_option(next[k], e, style_parsers);
                }
                // For the last option, pass the unparsed tokens
                // so that they can be added to next.back()'s values
                // if appropriate.
                finish_option(next.back(), args, style_parsers);
                for (unsigned j = 0; j < next.size(); ++j)
                    result.push_back(next[j]);
            }

            if (args.size() != current_size) {
                ok = true;
                break;
            }
        }

        if (!ok) {
            option opt;
            opt.value.push_back(args[0]);
            opt.original_tokens.push_back(args[0]);
            result.push_back(opt);
            args.erase(args.begin());
        }
    }

    /* If an key option is followed by a positional option,
       can can consume more tokens (e.g. it's multitoken option),
       give those tokens to it.  */
    std::vector<option> result2;
    for (unsigned i = 0; i < result.size(); ++i)
    {
        result2.push_back(result[i]);
        option& opt = result2.back();

        if (opt.string_key.empty())
            continue;

        const option_description* xd = nullptr;
        try
        {
            xd = m_options_description->find_nothrow(opt.string_key,
                is_style_active(allow_guessing),
                is_style_active(long_case_insensitive),
                is_style_active(short_case_insensitive));
        }
        catch(error_with_option_name& e)
        {
            // add context and rethrow
            e.add_context(opt.string_key, opt.original_tokens[0], get_canonical_option_prefix());
            throw;
        }

        if (!xd)
            continue;

        unsigned min_tokens = xd->semantic()->min_tokens();
        unsigned max_tokens = xd->semantic()->max_tokens();
        if (min_tokens < max_tokens && opt.value.size() < max_tokens)
        {
            // This option may grab some more tokens.
            // We only allow to grab tokens that are not already
            // recognized as key options.

            int can_take_more = max_tokens - static_cast<int>(opt.value.size());
            unsigned j = i+1;
            for (; can_take_more && j < result.size(); --can_take_more, ++j)
            {
                option& opt2 = result[j];
                if (!opt2.string_key.empty())
                    break;

                if (opt2.position_key == INT_MAX)
                {
                    // We use INT_MAX to mark positional options that
                    // were found after the '--' terminator and therefore
                    // should stay positional forever.
                    break;
                }

                assert(opt2.value.size() == 1);

                opt.value.push_back(opt2.value[0]);

                assert(opt2.original_tokens.size() == 1);

                opt.original_tokens.push_back(opt2.original_tokens[0]);
            }
            i = j-1;
        }
    }
    result.swap(result2);


    // Assign position keys to positional options.
    int position_key = 0;
    for(unsigned i = 0; i < result.size(); ++i) {
        if (result[i].string_key.empty())
            result[i].position_key = position_key++;
    }

    if (m_positional_options_description)
    {
        unsigned position = 0;
        for (unsigned i = 0; i < result.size(); ++i) {
            option& opt = result[i];
            if (opt.position_key != -1) {
                if (position >= m_positional_options_description->max_total_count())
                {
                    throw too_many_positional_options_error();
                }
                opt.string_key = m_positional_options_description->name_for_position(position);
                ++position;
            }
        }
    }

    // set case sensitive flag
    for (unsigned i = 0; i < result.size(); ++i) {
        if (result[i].string_key.size() > 2 ||
                (result[i].string_key.size() > 1 && result[i].string_key[0] != '-'))
        {
            // it is a long option
            result[i].case_insensitive = is_style_active(long_case_insensitive);
        }
        else
        {
            // it is a short option
            result[i].case_insensitive = is_style_active(short_case_insensitive);
        }
    }

    return result;
}

void command_line_parser::finish_option(option& opt,
    std::vector<std::string>& other_tokens,
    const std::vector<style_parser>& style_parsers)
{
    if (opt.string_key.empty())
        return;

    //
    // Be defensive:
    // will have no original token if option created by handle_additional_parser()
    std::string original_token_for_exceptions = opt.string_key;
    if (opt.original_tokens.size())
        original_token_for_exceptions = opt.original_tokens[0];

    try
    {
        // First check that the option is valid, and get its description.
        const option_description* xd = m_options_description->find_nothrow(
            opt.string_key,
            is_style_active(command_line_style::allow_guessing),
            is_style_active(command_line_style::long_case_insensitive),
            is_style_active(command_line_style::short_case_insensitive));

        if (!xd)
        {
            if (m_allow_unregistered) {
                opt.unregistered = true;
                return;
            } else {
                throw unknown_option();
            }
        }
        const option_description& d = *xd;

        // Canonize the name
        opt.string_key = d.key(opt.string_key);

        // We check that the min/max number of tokens for the option
        // agrees with the number of tokens we have. The 'adjacent_value'
        // (the value in --foo=1) counts as a separate token, and if present
        // must be consumed. The following tokens on the command line may be
        // left unconsumed.
        unsigned min_tokens = d.semantic()->min_tokens();
        unsigned max_tokens = d.semantic()->max_tokens();

        unsigned present_tokens = static_cast<unsigned>(opt.value.size() + other_tokens.size());

        if (present_tokens >= min_tokens)
        {
            if (!opt.value.empty() && max_tokens == 0)
            {
                throw invalid_command_line_syntax(invalid_command_line_syntax::extra_parameter);
            }

            // Grab min_tokens values from other_tokens, but only if those tokens
            // are not recognized as options themselves.
            if (opt.value.size() <= min_tokens)
            {
                min_tokens -= static_cast<unsigned>(opt.value.size());
            }
            else
            {
                min_tokens = 0;
            }

            // Everything's OK, move the values to the result.
            for(;!other_tokens.empty() && min_tokens--; )
            {
                // check if extra parameter looks like a known option
                // we use style parsers to check if it is syntactically an option,
                // additionally we check if an option_description exists
                std::vector<option> followed_option;
                std::vector<std::string> next_token(1, other_tokens[0]);
                for (unsigned i = 0; followed_option.empty() && i < style_parsers.size(); ++i)
                {
                    followed_option = style_parsers[i](next_token);
                }
                if (!followed_option.empty())
                {
                    original_token_for_exceptions = other_tokens[0];
                    const option_description* od = m_options_description->find_nothrow(other_tokens[0],
                            is_style_active(command_line_style::allow_guessing),
                            is_style_active(command_line_style::long_case_insensitive),
                            is_style_active(command_line_style::short_case_insensitive));
                    if (od) {
                        throw invalid_command_line_syntax(invalid_command_line_syntax::missing_parameter);
                    }
                }
                opt.value.push_back(other_tokens[0]);
                opt.original_tokens.push_back(other_tokens[0]);
                other_tokens.erase(other_tokens.begin());
            }
        }
        else
        {
            throw invalid_command_line_syntax(invalid_command_line_syntax::missing_parameter);

        }
    }
    // use only original token for unknown_option / ambiguous_option since by definition
    //    they are unrecognised / unparsable
    catch(error_with_option_name& e)
    {
        // add context and rethrow
        e.add_context(opt.string_key, original_token_for_exceptions, get_canonical_option_prefix());
        throw;
    }

}

std::vector<option>
command_line_parser::parse_long_option(std::vector<std::string>& args)
{
    std::vector<option> result;
    const std::string& tok = args[0];
    if (tok.size() >= 3 && tok[0] == '-' && tok[1] == '-')
    {
        std::string name, adjacent;

        std::string::size_type p = tok.find('=');
        if (p != tok.npos)
        {
            name = tok.substr(2, p-2);
            adjacent = tok.substr(p+1);
            if (adjacent.empty()) {
                throw invalid_command_line_syntax(
                    invalid_command_line_syntax::empty_adjacent_parameter,
                    name, name,
                    get_canonical_option_prefix());
            }
        }
        else
        {
            name = tok.substr(2);
        }
        option opt;
        opt.string_key = name;
        if (!adjacent.empty())
            opt.value.push_back(adjacent);
        opt.original_tokens.push_back(tok);
        result.push_back(opt);
        args.erase(args.begin());
    }
    return result;
}

std::vector<option>
command_line_parser::parse_short_option(std::vector<std::string>& args)
{
    const std::string& tok = args[0];
    if (tok.size() >= 2 && tok[0] == '-' && tok[1] != '-')
    {
        std::vector<option> result;

        std::string name = tok.substr(0,2);
        std::string adjacent = tok.substr(2);

        // Short options can be 'grouped', so that
        // "-d -a" becomes "-da". Loop, processing one
        // option at a time. We exit the loop when either
        // we've processed all the token, or when the remainder
        // of token is considered to be value, not further grouped
        // option.
        for(;;) {
            const option_description* d;
            try
            {
                d = m_options_description->find_nothrow(name, false, false,
                    is_style_active(command_line_style::short_case_insensitive));
            }
            catch(error_with_option_name& e)
            {
                // add context and rethrow
                e.add_context(name, name, get_canonical_option_prefix());
                throw;
            }

            // FIXME: check for 'allow_sticky'.
            if (d && (m_style & command_line_style::allow_sticky) &&
                    d->semantic()->max_tokens() == 0 && !adjacent.empty()) {
                // 'adjacent' is in fact further option.
                option opt;
                opt.string_key = name;
                result.push_back(opt);

                if (adjacent.empty())
                {
                    args.erase(args.begin());
                    break;
                }

                name = std::string("-") + adjacent[0];
                adjacent.erase(adjacent.begin());
            } else {
                option opt;
                opt.string_key = name;
                opt.original_tokens.push_back(tok);
                if (!adjacent.empty())
                    opt.value.push_back(adjacent);
                result.push_back(opt);
                args.erase(args.begin());
                break;
            }
        }
        return result;
    }
    return {};
}

std::vector<option>
command_line_parser::parse_dos_option(std::vector<std::string>& args)
{
    std::vector<option> result;
    const std::string& tok = args[0];
    if (tok.size() >= 2 && tok[0] == '/')
    {
        std::string name = "-" + tok.substr(1,1);
        std::string adjacent = tok.substr(2);

        option opt;
        opt.string_key = name;
        if (!adjacent.empty())
            opt.value.push_back(adjacent);
        opt.original_tokens.push_back(tok);
        result.push_back(opt);
        args.erase(args.begin());
    }
    return result;
}

std::vector<option>
command_line_parser::parse_disguised_long_option(std::vector<std::string>& args)
{
    const std::string& tok = args[0];
    if (tok.size() >= 2 &&
            ((tok[0] == '-' && tok[1] != '-') ||
             ((m_style & command_line_style::allow_slash_for_short) && tok[0] == '/')))
    {
        try
        {
            if (m_options_description->find_nothrow(
                tok.substr(1, tok.find('=')-1),
                is_style_active(command_line_style::allow_guessing),
                is_style_active(command_line_style::long_case_insensitive),
                is_style_active(command_line_style::short_case_insensitive)))
            {
                args[0].insert(0, "-");
                if (args[0][1] == '/')
                    args[0][1] = '-';
                return parse_long_option(args);
            }
        }
        catch(error_with_option_name& e)
        {
            // add context and rethrow
            e.add_context(tok, tok, get_canonical_option_prefix());
            throw;
        }
    }
    return std::vector<option>();
}

std::vector<option>
command_line_parser::parse_terminator(std::vector<std::string>& args)
{
    std::vector<option> result;
    const std::string& tok = args[0];
    if (tok == "--")
    {
        for(unsigned i = 1; i < args.size(); ++i)
        {
            option opt;
            opt.value.push_back(args[i]);
            opt.original_tokens.push_back(args[i]);
            opt.position_key = INT_MAX;
            result.push_back(opt);
        }
        args.clear();
    }
    return result;
}

int command_line_parser::get_canonical_option_prefix() const
{
    if (m_style & command_line_style::allow_long)
        return command_line_style::allow_long;

    if (m_style & command_line_style::allow_long_disguise)
        return command_line_style::allow_long_disguise;

    if ((m_style & command_line_style::allow_short) && (m_style & command_line_style::allow_dash_for_short))
        return command_line_style::allow_dash_for_short;

    if ((m_style & command_line_style::allow_short) && (m_style & command_line_style::allow_slash_for_short))
        return command_line_style::allow_slash_for_short;

    return 0;
}

bool command_line_parser::is_style_active(command_line_style::style_t style) const
{
    return ((m_style & style) ? true : false);
}

} // namespace program_options
