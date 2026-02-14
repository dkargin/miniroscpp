//
// Created by dkargin on 2/14/26.
//

#ifndef MINIROS_GTEST_PRINTERS_H
#define MINIROS_GTEST_PRINTERS_H

#include <ostream>

#include "miniros/errors.h"

#include "miniros/http/http_request.h"
#include "miniros/http/http_client.h"

/// This is collection of pretty printers for gtest.
namespace miniros {

inline void PrintTo(const Error& obj, ::std::ostream* os) {
  *os << obj.toString();
}

inline void PrintTo(const http::HttpClient::State& obj, ::std::ostream* os)
{
  *os << obj.toString();
}

inline void PrintTo(const http::HttpRequest::State& obj, ::std::ostream* os)
{
  *os << obj.toString();
}


}

#endif // MINIROS_GTEST_PRINTERS_H
