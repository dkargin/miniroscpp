//
// Created by dkargin on 8/13/26.
//

#ifndef MINIROS_BUILD_CONFIG_H
#define MINIROS_BUILD_CONFIG_H

#include "miniros/macros.h"

// Features present in this release. Older headers do not define these, so a
// caller can fall back with #ifdef after including any miniros header.
// Add a MINIROS_HAS_* / MINIBAG_HAS_* line here when a feature lands.

//! Forward bag scan that yields messages in file order.
#define MINIBAG_HAS_UNORDERED_VIEW 1

#ifdef __cplusplus

namespace miniros {

/**
 * Compile-time identity of this MiniROS tree.
 *
 * getApiVersionString() is header-only and reflects the headers this
 * translation unit was compiled against. getBinaryVersionString() and the
 * sanitizer / git accessors are baked into libroscxx. Compare the two version
 * strings to detect a header/library mismatch.
 */
struct BuildConfig {
  /// CMake project version of the headers (e.g. "0.7.0").
  static constexpr const char* getApiVersionString()
  {
    return MINIROS_VERSION_STRING;
  }

  /// CMake project version baked into the linked MiniROS binary.
  MINIROS_DECL static const char* getBinaryVersionString();

  /// True if the linked binary was built with AddressSanitizer.
  MINIROS_DECL static bool useASan();

  /// True if the linked binary was built with ThreadSanitizer.
  MINIROS_DECL static bool useTSan();

  /// Git commit hash of the library build, or empty if unavailable.
  MINIROS_DECL static const char* getGitCommit();

  /// Git branch of the library build, or empty if unavailable.
  MINIROS_DECL static const char* getGitBranch();
};

} // namespace miniros

#endif // __cplusplus

#endif
