#include <cassert>
#include <mutex>
#include <string>
#include <fstream>
#include <iomanip>
#include <malloc.h>
#include <thread>

#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <map>
#include "miniros/io/io.h"

#include "miniros/internal/profiling.h"

#include "static_write_buf.h"

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif

#ifdef PROFILE_NVTX_SUPPORT
#include <nvtx3/nvToolsExt.h>
#endif

#ifdef PROFILE_VTUNE_SUPPORT
#include <ittnotify.h>
#endif

namespace miniros {
namespace profiling {

int64_t currentThreadId()
{
  std::thread::id rowId = std::this_thread::get_id();
  long int threadId = 0;
  std::memcpy(&threadId, &rowId, sizeof(rowId));
  return threadId;
}

static int64_t GetCurrentTimeNanos() {
  const int64_t kNanosPerSecond = 1000 * 1000 * 1000;
#ifdef _WIN32
  // Windows implementation using QueryPerformanceCounter
  static LARGE_INTEGER frequency = {0};
  if (frequency.QuadPart == 0) {
    QueryPerformanceFrequency(&frequency);
    if (frequency.QuadPart == 0) {
      // This should not happen on Windows XP or later
      // Set to a safe default (1 MHz) to avoid division by zero
      frequency.QuadPart = 1000000;
    }
  }
  
  LARGE_INTEGER counter;
  QueryPerformanceCounter(&counter);
  // Convert to nanoseconds: (counter / frequency) * 1e9
  // Use 64-bit arithmetic to avoid precision loss
  return (counter.QuadPart * kNanosPerSecond) / frequency.QuadPart;
#else
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (int64_t{ts.tv_sec} * kNanosPerSecond +
          int64_t{ts.tv_nsec});
#endif
}

void RenderEvent(const TraceEvent& e, bool newLine, StaticWriteBuf& out) {
  if (newLine)
    out.puts(",\n");

  int pid = e.pid;
  if (pid == 0)
    pid = getpid();

  out.puts("{\"name\":"); out.putsq(e.name); out.puts(", ");
  out.puts("\"ph\":\""); out.putc(static_cast<char>(e.phase)); out.puts("\", ");
  out.puts("\"pid\":"); out.puti(pid); out.puts(", ");
  out.puts("\"tid\":"); out.putui64(e.tid); out.puts(", ");

  out.puts("\"cat\":"); out.putsq(e.categories); out.puts(", ");

  if (e.scope) {
    out.puts("\"scope\":"); out.putsq(e.scope); out.puts(", ");
  }

  out.puts("\"ts\":");
  out.puti(e.time);

  out.puts(", \"args\":{");

  bool first = true;

  for (const auto& kv : e.args_int) {
    if (!first)
      out.puts(", ");
    out.putsq(kv.first.c_str());
    out.puts(":");
    out.puti(kv.second);
    first = false;
  }

  for (const auto& kv : e.args_double) {
    if (!first) out.puts(", ");
    out.putsq(kv.first.c_str());
    out.puts(":");
    auto val = kv.second;
    if (!std::isfinite(val))
      val = 0;
    out.putd(kv.second);
    first = false;
  }

  for (const auto& kv : e.args_string) {
    if (!first) out.puts(", ");
    out.putsq(kv.first.c_str());
    out.puts(":");
    out.putsq(kv.second.c_str());
    first = false;
  }

  out.puts("}");
  out.puts("}");
}

struct ProfilingDomain::Internal {
#ifdef PROFILE_NVTX_SUPPORT
  nvtxDomainHandle_t nvtxDomain = nullptr;
#endif
#ifdef PROFILE_VTUNE_SUPPORT
  __itt_domain* ittDomain = nullptr;
#endif
  std::string name;
  /// Lock for file writing.
  mutable std::mutex traceMutex;
  FILE* traceDump = nullptr;

  mutable size_t lastNiceThreadId = 0;
  mutable std::map<int64_t, int> threadMap;

  /// Optional sink for additional trace consumers (e.g. ROS topic).
  ProfilingDomain::TraceSink traceSink;

  int64_t startTimeNanos = 0;

  int getGoodThreadId() const
  {
    int threadId = currentThreadId();
    std::unique_lock lock(traceMutex);
    auto it = threadMap.find(threadId);
    if (it == threadMap.end()) {
      threadMap[threadId] = ++lastNiceThreadId;
      return lastNiceThreadId;
    }
    return it->second;
  }

  Internal()
  {
    startTimeNanos = GetCurrentTimeNanos();
    const char* dump = getenv("MINIROS_TRACE");
    if (dump != nullptr) {
      int tid = getGoodThreadId();
      std::string traceName = dump;
      std::lock_guard<std::mutex> lock(traceMutex);
      traceDump = fopen(traceName.c_str(), "w");
      if (traceDump) {
        fputs("[\n", traceDump);
        TraceEvent evt;
        evt.name = "process_name";
        evt.phase = Phase::METADATA;
        evt.time = 0;
        evt.tid = tid;
        writeEventUnsafe(evt, true);
      } else {
        std::cerr << "Failed to open trace file \"" << traceName << "\"" << std::endl;
      }
    }
  }

  ~Internal()
  {
#ifdef PROFILE_NVTX_SUPPORT
    if (nvtxDomain) {
      nvtxDomainDestroy(nvtxDomain);
      nvtxDomain = nullptr;
    }
#endif
    std::lock_guard<std::mutex> lock(traceMutex);
    if (traceDump) {
      fputs("]\n", traceDump);
      fclose(traceDump);
      traceDump = nullptr;
    }
  }

  bool writeEventUnsafe(const TraceEvent& evt, bool first)
  {
    if (!traceDump && !traceSink)
      return false;

    char tmpBuf[2096];
    StaticWriteBuf out(tmpBuf, sizeof(tmpBuf));

    RenderEvent(evt, !first, out);
    if (traceDump) {
      fwrite(out.data(), sizeof(char), out.size(), traceDump);
    }
    if (traceSink) {
      traceSink(out.data(), out.size());
    }
    return true;
  }

  /// @returns true if event was written to file.
  bool writeEvent(const TraceEvent& evt)
  {
    char tmpBuf[2096];
    StaticWriteBuf out(tmpBuf, sizeof(tmpBuf));
    RenderEvent(evt, true, out);
    std::unique_lock<std::mutex> lock(traceMutex);
    if (traceDump) {
      fwrite(out.data(), sizeof(char), out.size(), traceDump);
    }
    if (traceSink) {
      traceSink(out.data(), out.size());
    }
    return traceDump || static_cast<bool>(traceSink);
  }
};

ProfilingDomain::ProfilingDomain(const char* name)
{
  m_internal = new Internal();
  m_internal->name = name ? name : "Unnamed";
#ifdef PROFILE_NVTX_SUPPORT
  m_internal->nvtxDomain = nvtxDomainCreateA(name);
#endif

#ifdef PROFILE_VTUNE_SUPPORT
  m_internal->ittDomain = __itt_domain_create(name);
#endif
}

ProfilingDomain::~ProfilingDomain()
{
  if (m_internal) {
    delete m_internal;
    m_internal = nullptr;
  }
}

int ProfilingDomain::niceThreadId() const
{
  if (m_internal)
    return m_internal->getGoodThreadId();
  return 0;
}

bool ProfilingDomain::isTraceActive() const
{
  return m_internal ? m_internal->traceDump != nullptr : false;
}

bool ProfilingDomain::writeEvent(TraceEvent& evt, const SteadyTime& stamp) const
{
  if (!m_internal) {
    return false;
  }
  int64_t evtTimeNanos = !stamp.is_zero() ? stamp.toNSec() : GetCurrentTimeNanos();
  evt.time = (evtTimeNanos - m_internal->startTimeNanos) / 1000;
  if (evt.tid == 0) {
    evt.tid = m_internal->getGoodThreadId();
  }
  return m_internal->writeEvent(evt);
}

void ProfilingDomain::setTraceSink(TraceSink sink)
{
  if (!m_internal) {
    return;
  }
  std::lock_guard<std::mutex> lock(m_internal->traceMutex);
  m_internal->traceSink = std::move(sink);
}

ProfilingLocation::ProfilingLocation(const char* name)
{
  m_name = name ? name : "Unnamed";
}

ProfilingLocation::ProfilingLocation(const char* object, const char* name)
{
  assert(name && object);
  if (name && object) {
    char tmpBuf[512];
    StaticWriteBuf out(tmpBuf, sizeof(tmpBuf));
    out.puts(object);
    out.puts("::");
    out.puts(name);
    m_name = out.data();
  }
}

const char* ProfilingLocation::name() const
{
  return m_name.c_str();
}


ProfilingScope::ProfilingScope(const ProfilingLocation* loc, const ProfilingDomain* domain)
    :  m_domain(domain), m_location(loc)
{
  assert(loc);
  if (loc) {
    const char* name = loc->name();
#ifdef PROFILE_NVTX_SUPPORT
    {
      nvtxRangePushA(name);
    }
#endif
#ifdef PROFILE_VTUNE_SUPPORT
{
  __itt_string_handle* task = __itt_string_handle_create(name);
  auto ittDomain = m_domain ? m_domain->getInternal()->ittDomain : nullptr;
  __itt_task_begin(ittDomain, __itt_null, __itt_null, task);
}
#endif
    if (m_domain && m_domain->isTraceActive()) {
      TraceEvent evt;
      evt.name = name;
      evt.phase = Phase::BEGIN;
      evt.categories = "perf";
      m_domain->writeEvent(evt);
    }
  }
}

ProfilingScope::~ProfilingScope()
{
#ifdef PROFILE_NVTX_SUPPORT
  nvtxRangePop();
#endif
#ifdef PROFILE_VTUNE_SUPPORT
  auto ittDomain = m_domain->getInternal()->ittDomain;
  __itt_task_end(ittDomain);
#endif

  if (m_domain && m_location && m_domain->isTraceActive()) {
    TraceEvent evt;
    evt.name = m_location->name();
    evt.phase = Phase::END;
    evt.categories = "perf";
    m_domain->writeEvent(evt);
  }
}

void mark([[maybe_unused]] ProfilingDomain* domain, const char* text)
{
  if (text == nullptr) {
    return;
  }
#ifdef PROFILE_NVTX_SUPPORT
  {
    nvtxEventAttributes_t eventAttrib = {0};
    eventAttrib.version = NVTX_VERSION;
    eventAttrib.size = NVTX_EVENT_ATTRIB_STRUCT_SIZE;
    eventAttrib.messageType = NVTX_MESSAGE_TYPE_ASCII;
    eventAttrib.message.ascii = text;
    auto nvtxDomain = domain ? domain->getInternal()->nvtxDomain : nullptr;
    nvtxDomainMarkEx(nvtxDomain, &eventAttrib);
  }
#endif

#ifdef PROFILE_VTUNE_SUPPORT
  auto string = __itt_string_handle_create(text);
  auto ittDomain = domain ? domain->getInternal()->ittDomain : nullptr;
  __itt_marker(ittDomain, __itt_null, string, __itt_scope::__itt_scope_task);
#endif
}

void mark([[maybe_unused]]const ProfilingDomain* domain, const char* object, const char* text)
{
  if (text == nullptr) {
    return;
  }

  char msg[512];
  StaticWriteBuf out(msg, sizeof(msg));

  out.puts(object);
  out.puts("::");
  out.puts(text);

#ifdef PROFILE_NVTX_SUPPORT
  {
    nvtxEventAttributes_t eventAttrib = {0};
    eventAttrib.version = NVTX_VERSION;
    eventAttrib.size = NVTX_EVENT_ATTRIB_STRUCT_SIZE;
    eventAttrib.messageType = NVTX_MESSAGE_TYPE_ASCII;
    eventAttrib.message.ascii = msg;
    auto nvtxDomain = domain ? domain->getInternal()->nvtxDomain : nullptr;
    nvtxDomainMarkEx(nvtxDomain, &eventAttrib);
  }
#endif
#ifdef PROFILE_VTUNE_SUPPORT
{
  auto string = __itt_string_handle_create(msg);
  auto ittDomain = domain ? domain->getInternal()->ittDomain : nullptr;
  __itt_marker(ittDomain, __itt_null, string, __itt_scope::__itt_scope_task);
}
#endif
}

ProfilingDomain g_profilingDomain("miniros");

void writeCurrentThreadNameInTrace(const std::string& name)
{
  TraceEvent event;
  event.name = "thread_name";
  event.phase = Phase::METADATA;
  event.time = 0;
  event.tid = g_profilingDomain.niceThreadId();
  event.pid = getpid();
  event.categories = "";
  event.args_string["name"] = name;

  g_profilingDomain.writeEvent(event);
}

} // namespace profiling
} // namespace miniros