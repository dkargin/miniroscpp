//
// Created by dkargin on 2/25/25.
//

#include "miniros/errors.h"

namespace miniros {

const char* Error::toString() const {
  switch(code) {
    case Error::Ok:
      return "Ok";
    case Error::NoMaster:
      return "No master";
    case Error::InvalidValue:
      return "Invalid value";
    case Error::SystemError:
      return "System error";
    case Error::NotImplemented:
      return "Not implemented";
    case Error::NotSupported:
      return "Not supported";
    case Error::ParameterNotFound:
      return "Parameter not found";
  }
  return "Unknown error";
}

} // namespace miniros
