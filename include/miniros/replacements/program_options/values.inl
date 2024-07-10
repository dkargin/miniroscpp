#ifndef MINIROS_PROGRAM_OPTIONS_VALIDATORS_INL
#define MINIROS_PROGRAM_OPTIONS_VALIDATORS_INL

#include <sstream>
#include <any>
#include <string>
#include <vector>

#include "errors.h"

namespace program_options {

namespace validators {
    /* If v.size() > 1, throw validation_error.
       If v.size() == 1, return v.front()
       Otherwise, returns a reference to a statically allocated
       empty string if 'allow_empty' and throws validation_error
       otherwise. */
    template<class charT>
    const std::basic_string<charT>& get_single_string(
        const std::vector<std::basic_string<charT> >& v,
        bool allow_empty = false)
    {
        static std::basic_string<charT> empty;
        if (v.size() > 1) {
            throw validation_error(validation_error::multiple_values_not_allowed);
        }
        else if (v.size() == 1)
            return v.front();
        else if (!allow_empty) {
            throw validation_error(validation_error::at_least_one_value_required);
        }
        return empty;
    }

    /* Throws multiple_occurrences if 'value' is not empty. */
    void check_first_occurrence(const std::any& value);
}

class bad_lexical_cast {

};

template <class T> T lexical_cast(std::string value) {
    std::istringstream ss(value);
    T result;
    ss >> result;
    if (ss.fail()) {
        throw bad_lexical_cast();
    }
    return result;
}

/** Validates 's' and updates 'v'.
    @pre 'v' is either empty or in the state assigned by the previous
    invocation of 'validate'.
    The target type is specified via a parameter which has the type of
    pointer to the desired type. This is workaround for compilers without
    partial template ordering, just like the last 'long/int' parameter.
*/
template<class T>
void validate(std::any& v, const std::vector<std::string>& xs, T*, long)
{
    validators::check_first_occurrence(v);
    std::string s(validators::get_single_string(xs));
    try {
        v = std::any(lexical_cast<T>(s));
    }
    catch(const bad_lexical_cast&) {
        throw invalid_option_value(s);
    }
}

void validate(std::any& v, const std::vector<std::string>& xs, bool*, int);

// For some reason, this declaration, which is require by the standard,
// cause msvc 7.1 to not generate code to specialization defined in
// value_semantic.cpp
void validate(std::any& v, const std::vector<std::string>& xs, std::string*, int);

/** Validates sequences. Allows multiple values per option occurrence
   and multiple occurrences. */
template<class T>
void validate(std::any& v, const std::vector<std::string>& s, std::vector<T>*, int)
{
    if (!v.has_value()) {
        v = std::any(std::vector<T>());
    }
    std::vector<T>* tv = std::any_cast< std::vector<T> >(&v);
    assert(NULL != tv);
    for (unsigned i = 0; i < s.size(); ++i)
    {
        try {
            /* We call validate so that if user provided
               a validator for class T, we use it even
               when parsing vector<T>.  */
            std::any a;
            std::vector<std::string> cv;
            cv.push_back(s[i]);
            validate(a, cv, (T*)0, 0);
            tv->push_back(std::any_cast<T>(a));
        }
        catch(const bad_lexical_cast& /*e*/) {
            throw invalid_option_value(s[i]);
        }
    }
}

/** Validates optional arguments. */
template<class T>
void validate(std::any& v, const std::vector<std::string>& s, std::optional<T>*, int)
{
    validators::check_first_occurrence(v);
    validators::get_single_string(s);
    std::any a;
    validate(a, s, (T*)0, 0);
    v = std::any(std::optional<T>(std::any_cast<T>(a)));
}

template <class T>
void typed_value<T>::xparse(std::any& value_store, const std::vector<std::string>& new_tokens) const
{
    // If no tokens were given, and the option accepts an implicit
    // value, then assign the implicit value as the stored value;
    // otherwise, validate the user-provided token(s).
    /*
    if (new_tokens.empty() && !m_implicit_value.empty())
        value_store = m_implicit_value;
    else
        validate(value_store, new_tokens, (T*)0, 0);
    */
    validate(value_store, new_tokens, (T*)0, 0);
}

} // namespace program_options

#endif // MINIROS_PROGRAM_OPTIONS_VALIDATORS_INL
