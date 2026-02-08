/// This is an internal file. It should not go to userspace, or it can break ABI.
///

#ifndef MINIROS_PROFILING_H
#define MINIROS_PROFILING_H

#include <map>
#include <functional>

#include "miniros/rostime.h"

namespace miniros {
class StaticWriteBuf;
template <class T> struct ImuData_;
/// A namespace for the instrumentation related to profiling.
/// These tools are simplifying integration with external tools like Intel VTune, NVidia NSight or RenderDoc.

namespace profiling {

enum class Phase: char {
  BEGIN = 'B',
  END = 'E',
  METADATA = 'M',
  COUNTER = 'C',
};

struct TraceEvent {
  const char* name = nullptr;
  Phase phase;
  uint64_t pid = 0;
  uint64_t tid = 0;
  uint64_t time = 0;
  const char* categories = nullptr;
  const char* process_name = nullptr;
  const char* scope = nullptr;

  std::map<std::string, int64_t> args_int;
  std::map<std::string, double> args_double;
  std::map<std::string, std::string> args_string;
};

/// Renders event to json line.
void RenderEvent(const TraceEvent& e, bool newLine, StaticWriteBuf& out);

/// A common domain for multiple profiling marks.
/// It can be used to group or distinguish profiling marks
/// from different parts of a larger project.
class ProfilingDomain {
public:
  // Wrapper for implementation-specific data.
  struct Internal;

  /// Callback type for receiving serialized JSON trace events.
  /// The buffer is exactly the JSON produced by RenderEvent (no null-terminator).
  using TraceSink = std::function<void(const char* data, size_t size)>;

  explicit ProfilingDomain(const char* name);
  ~ProfilingDomain();

  const Internal* getInternal() const
  {
    return m_internal;
  }

  /// Check if trace is active.
  bool isTraceActive() const;

  bool writeEvent(TraceEvent& evt, const SteadyTime& stamp = {}) const;

  /// Install a sink that will receive every serialized trace event.
  /// The sink is invoked under an internal mutex, so it must be fast
  /// and non-blocking to avoid stalling producers.
  void setTraceSink(TraceSink sink);

  /// Get nice thread ID of current thread.
  int niceThreadId() const;

private:
  Internal* m_internal = nullptr;
};

/// Encapsulates all info about unique location.
class ProfilingLocation {
public:
  ProfilingLocation(const char* name);
  ProfilingLocation(const char* object, const char* name);

  const char* name() const;

protected:
  std::string m_name;
};

/// Scoped profiler mark.
/// It will register start in constructor and end of the range in destructor.
class ProfilingScope {
public:
  ProfilingScope(const ProfilingLocation* location, const ProfilingDomain* domain);
  ~ProfilingScope();

protected:
  /// A raw pointer to a profiling domain.
  const ProfilingDomain* m_domain;
  /// A pointer to a location.
  /// Profiling system supposes that location is a static variable.
  const ProfilingLocation* m_location;
};

/// Places a simple mark.
void mark(const ProfilingDomain* domain, const char* mark);
/// Places a simple mark with additional object-specific name.
void mark(const ProfilingDomain* domain, const char* object, const char* mark);

extern ProfilingDomain g_profilingDomain;

/// Get ID of current thread.
int64_t currentThreadId();

void writeCurrentThreadNameInTrace(const std::string& name);

} // namespace profiling
} // namespace miniros

#define CONCAT3_INNER(a, b, c) a##b##c
#define CONCAT3(a, b, c) CONCAT3_INNER(a, b, c)

#define CONCAT2_INNER(a, b) a##b
#define CONCAT2(a, b) CONCAT2_INNER(a, b)

#define UNIQUE_NAME(base) CONCAT3(base, _ , __LINE__)

#ifdef MINIROS_USE_PROFILING

#define MINIROS_PROFILE_SCOPE(name) \
    static profiling::ProfilingLocation UNIQUE_NAME(_p_loc) (name); \
    profiling::ProfilingScope UNIQUE_NAME(_scope) (&UNIQUE_NAME(_p_loc), &profiling::g_profilingDomain);

#define MINIROS_PROFILE_SCOPE2(object, name) \
    static profiling::ProfilingLocation UNIQUE_NAME(_p_loc) (object, name); \
    profiling::ProfilingScope UNIQUE_NAME(_scope) (&UNIQUE_NAME(_p_loc), &profiling::g_profilingDomain);
#else

#define MINIROS_PROFILE_SCOPE(name)
#define MINIROS_PROFILE_SCOPE2(object, name)

#endif

#endif
