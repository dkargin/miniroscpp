//
// Created by dkargin on 8/13/26.
//

#include "miniros/build_config.h"
#include "internal_config.h"

namespace miniros {

const char* BuildConfig::getBinaryVersionString()
{
  return MINIROS_BINARY_VERSION_STRING;
}

bool BuildConfig::useASan()
{
#ifdef MINIROS_USE_ASAN
  return true;
#else
  return false;
#endif
}

bool BuildConfig::useTSan()
{
#ifdef MINIROS_USE_TSAN
  return true;
#else
  return false;
#endif
}

const char* BuildConfig::getGitCommit()
{
  return MINIROS_GIT_COMMIT;
}

const char* BuildConfig::getGitBranch()
{
  return MINIROS_GIT_BRANCH;
}

} // namespace miniros
