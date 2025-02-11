//
// Created by dkargin on 2/8/25.
//

#include <cstdlib>
#include <csignal>
#include <atomic>

#include "master.h"

std::atomic_bool g_sigintReceived {false};

void systemSignalHandler(int signal) {
  if (signal == SIGINT) {
    std::cout << "Got SIGINT. Stopping system" << std::endl;
    g_sigintReceived = true;
  }
}

int main(int argc, const char * argv[]) {
  std::signal(SIGINT, systemSignalHandler);

  miniros::Master master;

  master.start();

  while (!g_sigintReceived && master.ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  master.stop();

  return EXIT_SUCCESS;
}