//
// Created by dkargin on 2/23/25.
//

#ifndef MINIROS_ERRORS_H
#define MINIROS_ERRORS_H

#include "miniros/macros.h"

namespace miniros {

struct MINIROS_DECL Error {
  enum Error_t {
    /// Everything is ok.
    Ok,
    /// ROS mastre is not available.
    NoMaster,
    /// Input value is invalid.
    InvalidValue,
    /// Some generic system error,
    SystemError,
    /// Not implemented yet.
    NotImplemented,
    /// Implementation is disabled or not available for platform.
    NotSupported,
    /// Parameter was not found,
    ParameterNotFound,
    /// URI is invalid.
    InvalidURI,
  };

  Error(Error_t c) : code(c)
  {}

  operator bool() const
  {
    return code == Ok;
  }

  operator Error_t() const
  {
    return code;
  }

  Error& operator=(Error_t val)
  {
    code = val;
    return *this;
  }

  const char* toString() const;

  /// Error code.
  Error_t code;
};

} // namespac miniros

#endif //MINIROS_ERRORS_H
