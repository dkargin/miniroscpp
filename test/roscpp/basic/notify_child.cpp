//
// Helper process for Launcher tests.
// Usage: basic-notify_child [mode] [rpcPort] [uri]
// Modes: ready (default), crash, throw, loop, ignore, silent, say
//
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cctype>
#include <string>
#include <thread>
#include <chrono>
#include <stdexcept>

#include "miniros/common.h"

#if defined(WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <csignal>
#include <unistd.h>
#endif

namespace {

volatile bool g_keepRunning = true;

#if !defined(WIN32)
void onStopSignal(int)
{
  g_keepRunning = false;
}
#endif

void busyLoop()
{
  while (g_keepRunning) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

} // namespace

int main(int argc, char** argv)
{
  std::string mode = "ready";
  int argi = 1;
  if (argc > 1 && argv[1] && argv[1][0] && !std::isdigit(static_cast<unsigned char>(argv[1][0]))) {
    mode = argv[1];
    argi = 2;
  }

  miniros::NodeNotifyInfo info;
  if (argi < argc)
    info.rpcPort = std::atoi(argv[argi]);
  if (argi + 1 < argc)
    info.uri = argv[argi + 1];

  if (mode == "silent") {
    // Exit without notify — used to exercise waitReady() timeout.
    return 0;
  }

  if (mode == "say") {
    // Print distinct markers on stdout and stderr for redirectOutput() tests.
    const char* tag = (argi < argc && argv[argi] && argv[argi][0]) ? argv[argi] : "launcher-say";
    std::fprintf(stdout, "STDOUT:%s\n", tag);
    std::fprintf(stderr, "STDERR:%s\n", tag);
    std::fflush(stdout);
    std::fflush(stderr);
    return 0;
  }

  if (mode == "crash") {
    // Deliberate segfault for launcher crash handling.
    volatile int* p = nullptr;
    *p = 42;
    return 1;
  }

  if (mode == "throw") {
    throw std::runtime_error("launcher_child deliberate uncaught exception");
  }

  if (mode == "loop") {
#if !defined(WIN32)
    std::signal(SIGINT, onStopSignal);
    std::signal(SIGTERM, onStopSignal);
#endif
    (void)miniros::notifyNodeStarted(info);
    busyLoop();
    (void)miniros::notifyNodeExiting(info);
    return 0;
  }

  if (mode == "ignore") {
#if !defined(WIN32)
    std::signal(SIGINT, SIG_IGN);
    std::signal(SIGTERM, SIG_IGN);
    std::signal(SIGHUP, SIG_IGN);
    std::signal(SIGQUIT, SIG_IGN);
#endif
    (void)miniros::notifyNodeStarted(info);
    // Never clears g_keepRunning via signals; only SIGKILL / TerminateProcess stops this.
    for (;;) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  // Default: ready then exit.
  if (!miniros::notifyNodeStarted(info))
    return 2;
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  (void)miniros::notifyNodeExiting(info);
  return 0;
}
