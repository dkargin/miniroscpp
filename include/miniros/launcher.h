//
// Created by dkargin on 8/31/25.
//

#ifndef MINIROS_LAUNCHER_H
#define MINIROS_LAUNCHER_H

#include <string>
#include <vector>
#include <filesystem>


#include "miniros/macros.h"
#include "miniros/errors.h"

namespace miniros {

/// Launcher helps to starting applications and tracking its return status.
/// TODO: Make proper implementation for Win32. Right now it is just a placeholder code.
class MINIROS_DECL Launcher {
public:
  /// Create empty launcher. It can be start later.
  Launcher();
  /// Create launcher for specific PID.
  explicit Launcher(int64_t pid);

  ~Launcher();

  enum Flags {
    FLAG_DETACHED = (1 << 0),
  };

  Error start(const std::filesystem::path& path, const std::vector<std::string>& args, int flags);

  /// Add environment variables.
  Launcher& env(const char* name, const char* value);

  /// Send signal to an app.
  Error signal(int signal);

  /// Wait for exit.
  int waitExit();

  /// Get PID of a process.
  int64_t pid() const;

  /// Check if app is running.
  bool valid() const;

  static int64_t myPid();

protected:
  struct Internal;
  Internal *internal_;
};

} // namespace miniros
#endif // MINIROS_LAUNCHER_H
