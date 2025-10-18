//
// Created by dkargin on 2/8/25.
//

#include <cstdlib>
#include <csignal>
#include <atomic>
#include <filesystem>
#include <fstream>

#include "master.h"
#include "rosout.h"

#include "miniros/transport/callback_queue.h"
#include "miniros/xmlrpcpp/XmlRpcUtil.h"

#include "miniros/common.h"
#include "miniros/transport/io.h"
#include "miniros/transport/rpc_manager.h"

/// This define is injected in replacements/CMakeLists.txt
#ifdef USE_LOCAL_PROGRAM_OPTIONS
#include "program_options/program_options.h"
namespace po = program_options;
#else
#include "boost/program_options.hpp"
namespace po = boost::program_options;
#endif

using namespace miniros;

std::atomic_bool g_sigintReceived {false};

void systemSignalHandler(int signal) {
  if (signal == SIGINT) {
    std::cout << "Got SIGINT. Stopping system" << std::endl;
    g_sigintReceived = true;
  }
}

/// PidFile creates file with a PID of current process.
/// The file is removed in the destructor.
class PidFile {
public:
  PidFile() {}

  ~PidFile()
  {
    remove();
  }

  /// Create PID file.
  bool create(const char* file)
  {
    std::ofstream out(file);
    if (!out.is_open()) {
      std::cerr << "Failed to open PID file \"" << file << "\"" << std::endl;
    }
    out << getpid();
    out.close();
    path_ = std::filesystem::path(file);
    return true;
  }

  /// Remove PID file and clear internal state.
  void remove()
  {
    if (!path_.empty() && std::filesystem::is_regular_file(path_)) {
      std::filesystem::remove(path_);
    }
    path_.clear();
  }

protected:
  std::filesystem::path path_;
};

int main(int argc, const char ** argv) {
  std::signal(SIGINT, systemSignalHandler);

  SteadyTime timeStart = SteadyTime::now();

  po::options_description desc("Allowed options");

  desc.add_options()
    ("help,h", "produce help message")
    ("xmlrpc_log", po::value<int>()->default_value(1), "Verbosity level of XmlRpc logging")
    ("rosout", po::value<bool>()->default_value(true), "Enable rosout log aggregator")
    ("dir", po::value<std::string>(), "Path to working directory")
    ("resolve", po::value<bool>()->default_value(false), "Resolve node IP address")
    ("dump_parameters", po::value<bool>()->default_value(false), "Dump all ROSParam values on every update")
    // Unify rosmaster RPC manager and rosout RPC manager.
    // rosout creates its own RPC manager by default.
    ("unified_rpc", po::value<bool>()->default_value(false), "Resolve node IP address")
    ("pidfile", po::value<std::string>(), "Path to a PID file")
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

    if (!makeDirectory(wd))
      return EXIT_FAILURE;

    if (!changeCurrentDirectory(wd))
      return EXIT_FAILURE;
  }

  bool useRosout = vm.count("rosout") && vm["rosout"].as<bool>();
  if (vm.count("xmlrpc_log")) {
    int level = vm["xmlrpc_log"].as<int>();
    XmlRpc::setVerbosity(level);
  }

  bool dumpParameters = vm["dump_parameters"].as<bool>();

  std::shared_ptr<RPCManager> masterRpcManager = RPCManager::instance();

  MINIROS_INFO("Creating Master object");
  master::Master master(masterRpcManager);

  MINIROS_INFO("Initializing core transport");
  std::map<std::string, std::string> remappings;
  if (int port = master.getPort()) {
    remappings["__rpc_server_port"] = std::to_string(port);
  }

  init(remappings, "miniroscore", init_options::NoRosout | init_options::NoSigintHandler);

  master.setResolveNodeIP(resolve);
  master.setDumpParameters(dumpParameters);

  PidFile pidFile;
  if (vm.count("pidfile")) {
    pidFile.create(vm["pidfile"].as<std::string>().c_str());
  }

  MINIROS_INFO("Starting Master thread");

  // Start internal networking.
  if (Error err = start(); !err) {
    MINIROS_ERROR("Failed to start internal networking: %s", err.toString());
    return EXIT_FAILURE;
  }

  // This poll set is expected to be a global instance used by ROS. It is created by invoking `miniros::init` and exists
  // until application ends or miniros::shutdown() is invoked.
  PollSet* pollSet = masterRpcManager->getPollSet();
  if (!pollSet) {
    MINIROS_FATAL("Failed to get poll set");
    return EXIT_FAILURE;
  }

  if (!master.start(pollSet)) {
    MINIROS_ERROR("Failed to start Master");
    return EXIT_FAILURE;
  }

  NodeHandle node;

  std::unique_ptr<master::Rosout> r;
  if (useRosout) {
    MINIROS_INFO("Creating Rosout object");
    r.reset(new master::Rosout(node));
  }

  master.initEvents(node);

  CallbackQueue* callbackQueue = getGlobalCallbackQueue();
  if (useRosout && !callbackQueue) {
    return EXIT_FAILURE;
  }

  notifyNodeStarted();
  double durStartMs = (SteadyTime::now() - timeStart).toSec() * 1000.;
  MINIROS_INFO("All components have started in %fms", durStartMs);

  const WallDuration period(0.02);
  while (!g_sigintReceived && master.ok()) {
    if (callbackQueue)
      callbackQueue->callAvailable(period);
    else
      period.sleep();
    master.update();
  }

  MINIROS_INFO("Exiting main loop");

  notifyNodeExiting();
  r.reset();
  master.stop();

  MINIROS_INFO("All done");

  return EXIT_SUCCESS;
}