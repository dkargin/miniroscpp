//
// Created by dkargin on 2/25/25.
//

#include "miniros/errors.h"

namespace miniros {

const char* Error::toString() const {
  switch(code) {
    case Error::Ok:
      return "Ok";
    case Error::InternalError:
      return "Internal error";
    case Error::InvalidValue:
      return "Invalid value";
    case Error::OutOfMemory:
      return "Out of memory";

    case Error::NotImplemented:
      return "Not implemented";
    case Error::NotSupported:
      return "Not supported";
    case Error::ParameterNotFound:
      return "Parameter not found";
    case Error::NoMaster:
      return "No master";
    case Error::SystemError:
      return "System error";
    case Error::InvalidURI:
      return "Invalid URI";
    case Error::InvalidAddress:
      return "Invalid address";
    case Error::AddressInUse:
      return "Address in use";
    case Error::ShutdownInterrupt:
      return "Interrupted by shutdown request";
    case Error::InvalidResponse:
      return "Invalid response";
    case Error::EndOfFile:
      return "End of file";
    case Error::WouldBlock:
      return "Would block";
    case Error::FileNotFound:
      return "File not found";
    case Error::NotConnected:
      return "Not connected";
    case Error::InvalidHandle:
      return "Invalid handle";
    case Error::Timeout:
      return "Timeout";
    case Error::ResponsePostponed:
      return "ResponsePostponed";
    case Error::ResourceInUse:
      return "ResourceInUse";
  }
  return "Unknown error";
}

} // namespace miniros
