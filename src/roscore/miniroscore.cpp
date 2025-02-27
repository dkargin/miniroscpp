//
// Created by dkargin on 2/8/25.
//

#include <cstdlib>
#include <csignal>
#include <atomic>

#include "master.h"
#include "rosout.h"

#include "miniros/transport/callback_queue.h"
#include "miniros/xmlrpcpp/XmlRpcUtil.h"

/// This define is injected in replacements/CMakeLists.txt
#ifdef USE_LOCAL_PROGRAM_OPTIONS
#include "program_options/program_options.h"
namespace po = program_options;
#else
#include "boost/program_options.hpp"
namespace po = boost::program_options;
#endif

std::atomic_bool g_sigintReceived {false};

void systemSignalHandler(int signal) {
  if (signal == SIGINT) {
    std::cout << "Got SIGINT. Stopping system" << std::endl;
    g_sigintReceived = true;
  }
}

int main(int argc, const char ** argv) {
  std::signal(SIGINT, systemSignalHandler);

  po::options_description desc("Allowed options");

  desc.add_options()
    ("help,h", "produce help message")
    ("xmlrpc_log", po::value<int>()->default_value(1), "Verbosity level of XmlRpc logging")
    ("rosout", po::value<bool>()->default_value(true), "Enable rosout log aggregator")
    ;

  po::variables_map vm;

  try
  {
    po::store(po::command_line_parser(argc, argv)
              .options(desc)
              .run(), vm);
  }
  catch (po::invalid_command_line_syntax& e)
  {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  catch (po::unknown_option& e)
  {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  if (vm.count("help")) {
    std::cout << desc << std::endl;
    return EXIT_SUCCESS;
  }

  auto rpcManager = miniros::RPCManager::instance();
  miniros::master::Master master(rpcManager);

  bool useRosout = vm.count("rosout") && vm["rosout"].as<bool>();
  if (vm.count("xmlrpc_log")) {
    int level = vm["xmlrpc_log"].as<int>();
    XmlRpc::setVerbosity(level);
  }

  master.start();

  std::unique_ptr<miniros::master::Rosout> r;
  if (useRosout) {
    int argc_ = 1;
    char** argv_ = const_cast<char**>(argv);
    miniros::init(argc_, argv_, "rosout", miniros::init_options::NoRosout | miniros::init_options::NoSigintHandler);
  }

  miniros::CallbackQueue* callbackQueue = miniros::getGlobalCallbackQueue();
  if (!callbackQueue) {
    return EXIT_FAILURE;
  }
  miniros::notifyNodeStarted();

  miniros::WallDuration period(0.02);
  while (!g_sigintReceived && master.ok()) {
    callbackQueue->callAvailable(period);
  }

  miniros::notifyNodeExiting();
  master.stop();

  return EXIT_SUCCESS;
}