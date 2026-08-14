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

#include "miniros/callback_queue.h"
#include "miniros/io/io.h"

#include "miniros/common.h"
#include "miniros/transport/rpc_manager.h"

#include "miniros/xmlrpcpp/XmlRpcUtil.h" //< Needed to set log level.

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
      return false;
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
  // Install early so a crash during option parsing / init still dumps a stack.
  handleCrashes();
  std::signal(SIGINT, systemSignalHandler);

  SteadyTime timeStart = SteadyTime::now();

  po::options_description desc("Allowed options");

  desc.add_options()
    ("help,h", "produce help message")
    ("port,p", po::value<int>()->default_value(11311), "Server port")
    ("xmlrpc_log", po::value<int>()->default_value(1), "Verbosity level of XmlRpc logging")
    ("rosout", po::value<bool>()->default_value(true), "Enable rosout log aggregator")
    ("dir", po::value<std::string>(), "Path to working directory. This directory will be used for cache and other working files.")
    ("no-cache", "Disable persistent state cache (cache.<port> in cwd)")
    ("resolve", po::value<bool>()->default_value(false), "Resolve node IP address")
    ("dump_parameters", po::value<bool>()->default_value(false), "Dump all ROSParam values on every update")
    ("pidfile", po::value<std::string>(), "Path to a PID file")
    ("discovery", po::value<int>(), "UDP port for multimaster unicast sync (default: master RPC port)")
    ("multicast", po::value<std::string>()->default_value("239.255.42.42:11312"),
      "Multicast discovery group addr:port (use 'off' to disable)")
    ("token", po::value<std::string>(),
      "Shared secret for multimaster collective (optional; can also be entered in the web UI)")
    ("peer", po::value<std::vector<std::string>>()->composing(),
      "Peer master host:syncUdpPort to probe (repeatable; fallback when multicast is blocked)")
    ("node_check_period", po::value<double>()->default_value(5.0),
      "Period in seconds for checking whether registered nodes are still alive (0 disables)")
    ("debugAPI", "Enable debug HTTP API at /debugAPI/... (e.g. GET /debugAPI/shutdown)")
    ;

  po::variables_map vm;

  try
  {
    po::store(po::command_line_parser(argc, argv)
              .options(desc)
              .run(), vm);
  }
  catch (const po::error& e)
  {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
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

  std::string workingDir;
  if (vm.count("dir")) {
    workingDir = vm["dir"].as<std::string>();

    if (!makeDirectory(workingDir))
      return EXIT_FAILURE;

    if (!changeCurrentDirectory(workingDir))
      return EXIT_FAILURE;
  }

  bool useRosout = vm.count("rosout") && vm["rosout"].as<bool>();
  if (vm.count("xmlrpc_log")) {
    int level = vm["xmlrpc_log"].as<int>();
    miniros::XmlRpc::setVerbosity(level);
  }

  bool dumpParameters = vm["dump_parameters"].as<bool>();

  std::shared_ptr<RPCManager> masterRpcManager = RPCManager::instance();

  miniros::console::set_logger_level("miniros.http", console::Level::Info);
  miniros_console_bridge::setLogLevel(miniros_console_bridge::CONSOLE_BRIDGE_LOG_DEBUG);
  
  setThreadName("miniroscore");

  MINIROS_INFO("Creating Master object");
  master::Master master(masterRpcManager);

  MINIROS_INFO("Initializing core transport");
  std::map<std::string, std::string> remappings;

  int port = vm["port"].as<int>();
  if (port == 0) {
    port = 11311;
  }
  remappings["__rpc_server_port"] = std::to_string(port);
  remappings["__miniros.debug"] = "1";

  constexpr int options = init_options::LocalMaster | init_options::NoRosout | init_options::NoSigintHandler;
  init(remappings, "miniroscore", options);

  master.setResolveNodeIP(resolve);
  master.setDumpParameters(dumpParameters);
  master.setNodeCheckPeriod(vm["node_check_period"].as<double>());
  // Cache files (cache.<port>) are written to the current working directory.
  master.setCacheEnabled(!vm.count("no-cache"));
  if (vm.count("debugAPI"))
    master.setDebugApi(true);

  PidFile pidFile;
  if (vm.count("pidfile")) {
    pidFile.create(vm["pidfile"].as<std::string>().c_str());
  }

  if (vm.count("token")) {
    master.setMultimasterToken(vm["token"].as<std::string>());
  }

  if (vm.count("discovery")) {
    master.setMultimasterUdpPort(vm["discovery"].as<int>());
  }

  if (vm.count("multicast")) {
    const std::string mc = vm["multicast"].as<std::string>();
    if (mc.empty() || mc == "off" || mc == "none") {
      master.setMultimasterMulticast("", 0);
    } else {
      auto colon = mc.rfind(':');
      if (colon == std::string::npos) {
        MINIROS_ERROR("Invalid --multicast \"%s\", expected addr:port or 'off'", mc.c_str());
        return EXIT_FAILURE;
      }
      std::string host = mc.substr(0, colon);
      int mcPort = 0;
      try {
        mcPort = std::stoi(mc.substr(colon + 1));
      } catch (...) {
        MINIROS_ERROR("Invalid --multicast port in \"%s\"", mc.c_str());
        return EXIT_FAILURE;
      }
      master.setMultimasterMulticast(host, mcPort);
    }
  }

  if (vm.count("peer")) {
    for (const std::string& peer : vm["peer"].as<std::vector<std::string>>()) {
      auto colon = peer.rfind(':');
      if (colon == std::string::npos) {
        MINIROS_ERROR("Invalid --peer \"%s\", expected host:port", peer.c_str());
        return EXIT_FAILURE;
      }
      std::string host = peer.substr(0, colon);
      int peerPort = 0;
      try {
        peerPort = std::stoi(peer.substr(colon + 1));
      } catch (...) {
        MINIROS_ERROR("Invalid --peer port in \"%s\"", peer.c_str());
        return EXIT_FAILURE;
      }
      if (Error err = master.addMultimasterPeer(host, peerPort); !err) {
        MINIROS_ERROR("Failed to add multimaster peer \"%s\": %s", peer.c_str(), err.toString());
        return EXIT_FAILURE;
      }
    }
  }

  MINIROS_INFO("Starting Master thread");

  // Start internal networking.
  if (Error err = miniros::start(); !err) {
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

  if (!master.start(pollSet, port)) {
    MINIROS_ERROR("Failed to start Master");
    return EXIT_FAILURE;
  }

  NodeHandle node;

  std::unique_ptr<master::Rosout> r;
  if (useRosout) {
    MINIROS_INFO("Creating Rosout object");
    // Register /miniroscore as NODE_LOCAL before Rosout attaches pubs/subs.
    master.registerSelfRef();
    r.reset(new master::Rosout(node));
  }

  master.initEvents(node);

  CallbackQueue* callbackQueue = getGlobalCallbackQueue();
  if (useRosout && !callbackQueue) {
    return EXIT_FAILURE;
  }

  NodeNotifyInfo started;
  started.rpcPort = master.getPort();
  started.uri = master.getUri();
  notifyNodeStarted(started);
  double durStartMs = (SteadyTime::now() - timeStart).toSec() * 1000.;
  MINIROS_INFO("All components have started in %fms. URL=%s", durStartMs, master.getUri().c_str());

  const WallDuration period(0.02);
  while (!g_sigintReceived && master.ok()) {
    if (callbackQueue)
      callbackQueue->callAvailable(period);
    else
      period.sleep();
    master.update();
  }

  MINIROS_INFO("Exiting main loop");

  NodeNotifyInfo stopping;
  stopping.rpcPort = master.getPort();
  stopping.uri = master.getUri();
  notifyNodeExiting(stopping);
  r.reset();
  master.stop();

  MINIROS_INFO("All done");

  return EXIT_SUCCESS;
}