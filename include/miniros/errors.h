//
// Created by dkargin on 2/23/25.
//

#ifndef MINIROS_ERRORS_H
#define MINIROS_ERRORS_H

namespace miniros {

struct Error {
  enum Error_t {
    /// Everything is ok.
    Ok,
    /// ROS mastre is not available.
    NoMaster,
    /// Input value is invalid.
    InvalidValue,

    NotImplemented,
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
