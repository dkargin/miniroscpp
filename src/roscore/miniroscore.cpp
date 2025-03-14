//
// Created by dkargin on 2/8/25.
//

#include <cstdlib>
#include <csignal>
#include <atomic>
#include <filesystem>

#include "master.h"
#include "rosout.h"

#include "miniros/transport/callback_queue.h"
#include "miniros/xmlrpcpp/XmlRpcUtil.h"

#include "miniros/common.h"

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
    ("dir", po::value<std::string>(), "Path to working directory")
    ("resolve", po::value<bool>()->default_value(false), "Resolve node IP address")
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

  bool resolve = false;
  if (vm.count("resolve")) {
    resolve = vm["resolve"].as<bool>();
  }

  if (vm.count("dir")) {
    std::string wd = vm["dir"].as<std::string>();

    if (!miniros::makeDirectory(wd))
      return EXIT_FAILURE;

    if (!miniros::changeCurrentDirectory(wd))
      return EXIT_FAILURE;
  }

  bool useRosout = vm.count("rosout") && vm["rosout"].as<bool>();
  if (vm.count("xmlrpc_log")) {
    int level = vm["xmlrpc_log"].as<int>();
    XmlRpc::setVerbosity(level);
  }

  MINIROS_INFO("Creating RPCManager");

  // Standalone RPC manger for rosmaster.
  auto masterRpcManager = std::make_shared<miniros::RPCManager>();

  MINIROS_INFO("Creating Master object");
  miniros::master::Master master(masterRpcManager);

  master.setResolveNodeIP(resolve);

  MINIROS_INFO("Starting Master thread");
  master.start();

  std::unique_ptr<miniros::master::Rosout> r;
  if (useRosout) {
    std::map<std::string, std::string> remappings;

    remappings["__master"] = master.getUri();

    MINIROS_INFO("Creating Rosout object");
    miniros::init(remappings, "rosout", miniros::init_options::NoRosout | miniros::init_options::NoSigintHandler);
    r.reset(new miniros::master::Rosout());
  }

  miniros::CallbackQueue* callbackQueue = miniros::getGlobalCallbackQueue();
  if (!callbackQueue) {
    return EXIT_FAILURE;
  }

  MINIROS_INFO("All components have started");
  miniros::notifyNodeStarted();

  miniros::WallDuration period(0.02);
  while (!g_sigintReceived && master.ok()) {
    callbackQueue->callAvailable(period);
    master.update();
  }

  MINIROS_INFO("Exiting main loop");

  miniros::notifyNodeExiting();
  r.reset();
  master.stop();

  MINIROS_INFO("All done");

  return EXIT_SUCCESS;
}