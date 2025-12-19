//
// Created by dkargin on 2/23/25.
//

#ifndef MINIROS_ERRORS_H
#define MINIROS_ERRORS_H

#include "miniros/macros.h"

namespace miniros {

struct MINIROS_DECL Error {
  /// Enumeration of all errors in project.
  /// NOTE: Do not forget to modify .toString() method after adding new values here.
  enum Error_t {
    /// Everything is ok.
    Ok,
    /// Internal error.
    /// In most cases it is related to missing internal_ object or some other kind
    /// of unexpected values of internal pointers. This is some serious assert-level problem.
    InternalError,
    /// Input value is invalid.
    InvalidValue,
    /// Not enough memory to finish operation.
    OutOfMemory,
    /// Not implemented yet.
    NotImplemented,
    /// Implementation is disabled or not available for platform.
    NotSupported,
    /// Parameter was not found.
    ParameterNotFound,
    /// ROS master is not available.
    NoMaster,
    /// Some generic error during system call.
    SystemError,
    /// URI is invalid.
    InvalidURI,
    /// Network address is invalid.
    InvalidAddress,
    /// Network address in use. Matches EADDRINUSE error.
    AddressInUse,
    /// Operation was interrupted by a shutdown request.
    ShutdownInterrupt,
    /// Incorrect or unexpected response.
    InvalidResponse,
    /// End of file or socket is closed.
    EndOfFile,
    /// Next IO attempt will block.
    WouldBlock,
    /// File not found or no corresponding endpoint.
    FileNotFound,
    /// Socket is not connected.
    NotConnected,
    /// Handle or file descriptor is invalid.
    InvalidHandle,
    /// Timed out during operation.
    Timeout,
    /// Response was postponed.
    Postponed,
  };

  Error() :code(Ok)
  {}

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
