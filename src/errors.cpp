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
    case Error::InvalidURI:
      return "Invalid URI";
    case Error::InternalError:
      return "Internal error";
    case Error::ShutdownInterrupt:
      return "Interrupted by shutdown request";
    case Error::InvalidResponse:
      return "Invalid response";
    case Error::AddressInUse:
      return "Address in use";
    case Error::EndOfFile:
      return "End of file";
    case Error::WouldBlock:
      return "Would block";
  }
  return "Unknown error";
}

} // namespace miniros
