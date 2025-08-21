//
// Created by dkargin on 3/1/25.
//

#include <string>
#include <sstream>
#include <fstream>
#include <regex> // to catch std::regex_error

#include "../replacements/progress_display.h"

#include "minibag/player.h"
#include "minibag/recorder.h"
#include "minibag/view.h"

#include "miniros/console.h"

/// This define is injected in replacements/CMakeLists.txt
#ifdef USE_LOCAL_PROGRAM_OPTIONS
#include "program_options/program_options.h"
namespace po = program_options;
#else
#include "boost/program_options.hpp"
namespace po = boost::program_options;
#endif


minibag::PlayerOptions parsePlayOptions(int argc, char** argv) {
  minibag::PlayerOptions opts;

    po::options_description desc("Allowed options");

    desc.add_options()
      ("help,h", "produce help message")
      ("prefix,p", po::value<std::string>()->default_value(""), "prefixes all output topics in replay")
      ("quiet,q", "suppress console output")
      ("immediate,i", "play back all messages without waiting")
      ("pause", "start in paused mode")
      ("queue", po::value<int>()->default_value(100), "use an outgoing queue of size SIZE")
      ("clock", "publish the clock time")
      ("hz", po::value<float>()->default_value(100.0f), "use a frequency of HZ when publishing clock time")
      ("delay,d", po::value<float>()->default_value(0.2f), "sleep SEC seconds after every advertise call (to allow subscribers to connect)")
      ("rate,r", po::value<float>()->default_value(1.0f), "multiply the publish rate by FACTOR")
      ("start,s", po::value<float>()->default_value(0.0f), "start SEC seconds into the bag files")
      ("duration,u", po::value<float>(), "play only SEC seconds from the bag files")
      ("skip-empty", po::value<float>(), "skip regions in the bag with no messages for more than SEC seconds")
      ("loop,l", "loop playback")
      ("keep-alive,k", "keep alive past end of bag (useful for publishing latched topics)")
      ("try-future-version", "still try to open a bag file, even if the version is not known to the player")
      ("topics", po::value< std::vector<std::string> >()->multitoken(), "topics to play back")
      ("pause-topics", po::value< std::vector<std::string> >()->multitoken(), "topics to pause playback on")
      ("bags", po::value< std::vector<std::string> >(), "bag files to play back from")
      ("wait-for-subscribers", "wait for at least one subscriber on each topic before publishing")
      ("rate-control-topic", po::value<std::string>(), "watch the given topic, and if the last publish was more than <rate-control-max-delay> ago, wait until the topic publishes again to continue playback")
      ("rate-control-max-delay", po::value<float>()->default_value(1.0f), "maximum time difference from <rate-control-topic> before pausing")
      ("ignore-topics", po::value< std::vector<std::string> >()->multitoken(), "topics to be skipped if playing whole bag")
      ;

    po::positional_options_description p;
    p.add("bags", -1);

    po::variables_map vm;

    try
    {
      po::store(po::command_line_parser(argc, argv)
                .options(desc)
                .positional(p)
                .run(), vm);
    }
    catch (po::invalid_command_line_syntax& e)
    {
      throw miniros::Exception(e.what());
    }
    catch (po::unknown_option& e)
    {
      throw miniros::Exception(e.what());
    }

    if (vm.count("help")) {
      std::cout << desc << std::endl;
      exit(0);
    }

    if (vm.count("prefix"))
      opts.prefix = vm["prefix"].as<std::string>();
    if (vm.count("quiet"))
      opts.quiet = true;
    if (vm.count("immediate"))
      opts.at_once = true;
    if (vm.count("pause"))
      opts.start_paused = true;
    if (vm.count("queue"))
      opts.queue_size = vm["queue"].as<int>();
    if (vm.count("hz"))
      opts.bag_time_frequency = vm["hz"].as<float>();
    if (vm.count("clock"))
      opts.bag_time = true;
    if (vm.count("delay"))
      opts.advertise_sleep = miniros::WallDuration(vm["delay"].as<float>());
    if (vm.count("rate"))
      opts.time_scale = vm["rate"].as<float>();
    if (vm.count("start"))
    {
      opts.time = vm["start"].as<float>();
      opts.has_time = true;
    }
    if (vm.count("duration"))
    {
      opts.duration = vm["duration"].as<float>();
      opts.has_duration = true;
    }
    if (vm.count("skip-empty"))
      opts.skip_empty = miniros::Duration(vm["skip-empty"].as<float>());
    if (vm.count("loop"))
      opts.loop = true;
    if (vm.count("keep-alive"))
      opts.keep_alive = true;
    if (vm.count("wait-for-subscribers"))
      opts.wait_for_subscribers = true;

    if (vm.count("ignore-topics")) {
      std::vector<std::string> ignore_topics = vm["ignore-topics"].as< std::vector<std::string> >();
      for (const std::string& topic: ignore_topics) {
        opts.ignore_topics.insert(topic);
      }
    }

    if (vm.count("topics"))
    {
      std::vector<std::string> topics = vm["topics"].as< std::vector<std::string> >();
      for (const std::string& topic: topics)
        opts.topics.push_back(topic);
    }

    if (vm.count("pause-topics"))
    {
      std::vector<std::string> pause_topics = vm["pause-topics"].as< std::vector<std::string> >();
      for (const std::string& topic: pause_topics)
        opts.pause_topics.push_back(topic);
    }

    if (vm.count("rate-control-topic"))
      opts.rate_control_topic = vm["rate-control-topic"].as<std::string>();

    if (vm.count("rate-control-max-delay"))
      opts.rate_control_max_delay = vm["rate-control-max-delay"].as<float>();

    if (vm.count("bags"))
    {
      std::vector<std::string> bags = vm["bags"].as< std::vector<std::string> >();
      for (const std::string& bag: bags)
          opts.bags.push_back(bag);
    } else {
      if (vm.count("topics") || vm.count("pause-topics"))
        throw miniros::Exception("When using --topics or --pause-topics, --bags "
          "should be specified to list bags.");
      throw miniros::Exception("You must specify at least one bag to play back.");
    }

    return opts;
}

/// Handles command `minibag play ...`
int runPlay(int argc, char* argv[])
{
  // Parse the command-line options
  minibag::PlayerOptions opts;

  try {
    opts = parsePlayOptions(argc, argv);
  }
  catch (miniros::Exception const& ex) {
    std::cerr << "Error reading options: " << ex.what();
    return EXIT_FAILURE;
  }

  minibag::Player player(opts);

  try {
    player.publish();
  }
  catch (std::runtime_error& e) {
    std::cerr << "Caught an exception: " << e.what();
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}


//! Parse the command-line arguments for recorder options
minibag::RecorderOptions parseRecordOptions(int argc, char** argv) {
    minibag::RecorderOptions opts;

    po::options_description desc("Allowed options");

    desc.add_options()
      ("help,h", "produce help message")
      ("all,a", "record all topics")
      ("regex,e", "match topics using regular expressions")
      ("exclude,x", po::value<std::string>(), "exclude topics matching regular expressions")
      ("quiet,q", "suppress console output")
      ("publish,p", "Publish a msg when the record begin")
      ("output-prefix,o", po::value<std::string>(), "prepend PREFIX to beginning of bag name")
      ("output-name,O", po::value<std::string>(), "record bagnamed NAME.bag")
      ("buffsize,b", po::value<int>()->default_value(256), "Use an internal buffer of SIZE MB (Default: 256)")
      ("chunksize", po::value<int>()->default_value(768), "Set chunk size of message data, in KB (Default: 768. Advanced)")
      ("limit,l", po::value<int>()->default_value(0), "Only record NUM messages on each topic")
      ("min-space,L", po::value<std::string>()->default_value("1G"), "Minimum allowed space on recording device (use G,M,k multipliers)")
      ("bz2,j", "use BZ2 compression")
      ("lz4", "use LZ4 compression")
      ("split", po::value<int>()->implicit_value(0), "Split the bag file and continue recording when maximum size or maximum duration reached.")
      ("max-splits", po::value<int>(), "Keep a maximum of N bag files, when reaching the maximum erase the oldest one to keep a constant number of files.")
      ("topic", po::value< std::vector<std::string> >(), "topic to record")
      ("size", po::value<uint64_t>(), "The maximum size of the bag to record in MB.")
      ("duration", po::value<std::string>(), "Record a bag of maximum duration in seconds, unless 'm', or 'h' is appended.")
      ("node", po::value<std::string>(), "Record all topics subscribed to by a specific node.")
      ("tcpnodelay", "Use the TCP_NODELAY transport hint when subscribing to topics.")
      ("udp", "Use the UDP transport hint when subscribing to topics.")
      ("topic-list", po::value< std::vector<std::string> >()->multitoken(), "path to a file with list of topics to record");


    po::positional_options_description p;
    p.add("topic", -1);

    po::variables_map vm;

    try
    {
      po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
    } catch (po::invalid_command_line_syntax& e)
    {
      throw miniros::Exception(e.what());
    }  catch (po::unknown_option& e)
    {
      throw miniros::Exception(e.what());
    }

    if (vm.count("help")) {
      std::cout << desc << std::endl;
      exit(0);
    }

    if (vm.count("all"))
      opts.record_all = true;
    if (vm.count("regex"))
      opts.regex = true;
    if (vm.count("exclude"))
    {
      opts.do_exclude = true;
      opts.exclude_regex = vm["exclude"].as<std::string>();
    }
    if (vm.count("quiet"))
      opts.quiet = true;
    if (vm.count("publish"))
      opts.publish = true;
    if (vm.count("output-prefix"))
    {
      opts.prefix = vm["output-prefix"].as<std::string>();
      opts.append_date = true;
    }
    if (vm.count("output-name"))
    {
      opts.prefix = vm["output-name"].as<std::string>();
      opts.append_date = false;
    }
    if (vm.count("split"))
    {
      opts.split = true;

      int S = vm["split"].as<int>();
      if (S != 0)
      {
        MINIROS_WARN("Use of \"--split <MAX_SIZE>\" has been deprecated.  Please use --split --size <MAX_SIZE> or --split --duration <MAX_DURATION>");
        if (S < 0)
          throw miniros::Exception("Split size must be 0 or positive");
        opts.max_size = 1048576 * static_cast<uint64_t>(S);
      }
    }
    if(vm.count("max-splits"))
    {
        if(!opts.split)
        {
            MINIROS_WARN("--max-splits is ignored without --split");
        }
        else
        {
            opts.max_splits = vm["max-splits"].as<int>();
        }
    }
    if (vm.count("buffsize"))
    {
      int m = vm["buffsize"].as<int>();
      if (m < 0)
        throw miniros::Exception("Buffer size must be 0 or positive");
      opts.buffer_size = 1048576 * m;
    }
    if (vm.count("chunksize"))
    {
      int chnk_sz = vm["chunksize"].as<int>();
      if (chnk_sz < 0)
        throw miniros::Exception("Chunk size must be 0 or positive");
      opts.chunk_size = 1024 * chnk_sz;
    }
    if (vm.count("limit"))
    {
      opts.limit = vm["limit"].as<int>();
    }
    if (vm.count("min-space"))
    {
        std::string ms = vm["min-space"].as<std::string>();
        long long int value = 1073741824ull;
        char mul = 0;
        // Sane default values, just in case
        opts.min_space_str = "1G";
        opts.min_space = value;
        if (sscanf(ms.c_str(), " %lld%c", &value, &mul) > 0) {
            opts.min_space_str = ms;
            switch (mul) {
                case 'G':
                case 'g':
                    opts.min_space = value * 1073741824ull;
                    break;
                case 'M':
                case 'm':
                    opts.min_space = value * 1048576ull;
                    break;
                case 'K':
                case 'k':
                    opts.min_space = value * 1024ull;
                    break;
                default:
                    opts.min_space = value;
                    break;
            }
        }
        MINIROS_DEBUG("Rosbag using minimum space of %lld bytes, or %s", opts.min_space, opts.min_space_str.c_str());
    }
    if (vm.count("bz2") && vm.count("lz4"))
    {
      throw miniros::Exception("Can only use one type of compression");
    }
    if (vm.count("bz2"))
    {
      opts.compression = minibag::compression::BZ2;
    }
    if (vm.count("lz4"))
    {
      opts.compression = minibag::compression::LZ4;
    }
    if (vm.count("duration"))
    {
      std::string duration_str = vm["duration"].as<std::string>();

      double duration;
      double multiplier = 1.0;
      std::string unit("");

      std::istringstream iss(duration_str);
      if ((iss >> duration).fail())
        throw miniros::Exception("Duration must start with a floating point number.");

      if ( (!iss.eof() && ((iss >> unit).fail())) )
      {
        throw miniros::Exception("Duration unit must be s, m, or h");
      }
      if (unit == std::string(""))
        multiplier = 1.0;
      else if (unit == std::string("s"))
        multiplier = 1.0;
      else if (unit == std::string("m"))
        multiplier = 60.0;
      else if (unit == std::string("h"))
        multiplier = 3600.0;
      else
        throw miniros::Exception("Duration unit must be s, m, or h");


      opts.max_duration = miniros::Duration(duration * multiplier);
      if (opts.max_duration <= miniros::Duration(0))
        throw miniros::Exception("Duration must be positive.");
    }
    if (vm.count("size"))
    {
      opts.max_size = vm["size"].as<uint64_t>() * 1048576;
      if (opts.max_size <= 0)
        throw miniros::Exception("Split size must be 0 or positive");
    }
    if (vm.count("node"))
    {
      opts.node = vm["node"].as<std::string>();
      std::cout << "Recording from: " << opts.node << std::endl;
    }
    if (vm.count("tcpnodelay"))
    {
      opts.transport_hints.tcpNoDelay();
    }
    if (vm.count("udp"))
    {
      opts.transport_hints.udp();
    }

    std::vector<std::string> topics;
    if (vm.count("topic-list")) {
      std::vector<std::string> listFiles = vm["topic-list"].as< std::vector<std::string> >();
      for (const std::string& file: listFiles) {
        if (!miniros::names::readTopicList(file, topics)) {
          std::cerr << "Failed to open file with topic list \"" << file.c_str() << "\"" << std::endl;
        }
      }
    }

    // Every non-option argument is assumed to be a topic
    if (vm.count("topic"))
    {
      std::vector<std::string> t = vm["topic"].as< std::vector<std::string> >();
      for (const std::string& topic: t)
        topics.push_back(topic);
    }

    std::sort(topics.begin(), topics.end());
    topics.erase(std::unique(topics.begin(), topics.end()), topics.end());
    opts.topics = topics;

    // check that argument combinations make sense
    if(opts.exclude_regex.mark_count() > 0 &&
            !(opts.record_all || opts.regex)) {
        std::cerr << "Warning: Exclusion regex given, but no topics to subscribe to.\n"
                    "Adding implicit 'record all'.";
        opts.record_all = true;
    }

    return opts;
}

/// Handles command `minibag record ...`
int runRecord(int argc, char** argv) {
  // Parse the command-line options
  minibag::RecorderOptions opts;
  try {
    opts = parseRecordOptions(argc, argv);
  }
  catch (miniros::Exception const& ex) {
    MINIROS_ERROR("Error reading options: %s", ex.what());
    return 1;
  }
  catch(std::regex_error const& ex) {
    MINIROS_ERROR("Error reading options: %s\n", ex.what());
    return 1;
  }

  // Run the recorder
  minibag::Recorder recorder(opts);
  int result = recorder.run();

  return result;
}

struct EncryptorOptions
{
    EncryptorOptions() : quiet(false), compression(minibag::compression::Uncompressed) { }

    void buildOutbagName();

    bool quiet;
    std::string plugin;
    std::string param;
    minibag::CompressionType compression;
    std::string inbag;
    std::string outbag;
};

void EncryptorOptions::buildOutbagName()
{
    if (!outbag.empty())
        return;
    if (inbag.empty())
        throw miniros::Exception("Input bag is not specified.");
    std::string::size_type pos = inbag.find_last_of('.');
    if (pos == std::string::npos)
        throw miniros::Exception("Input bag name has no extension.");
    outbag = inbag.substr(0, pos) + std::string(".out") + inbag.substr(pos);
}

//! Parse the command-line arguments for encrypt options
EncryptorOptions parseEncryptOptions(int argc, char** argv)
{
    EncryptorOptions opts;

    po::options_description desc("Allowed options");

    desc.add_options()
      ("help,h",   "produce help message")
      ("quiet,q",  "suppress console output")
      ("plugin,p", po::value<std::string>()->default_value("rosbag/AesCbcEncryptor"), "encryptor name")
      ("param,r",  po::value<std::string>()->default_value("*"), "encryptor parameter")
      ("bz2,j",    "use BZ2 compression")
      ("lz4",      "use lz4 compression")
      ("inbag",    po::value<std::string>(), "bag file to encrypt")
      ("outbag,o", po::value<std::string>(), "bag file encrypted")
      ;

    po::positional_options_description p;
    p.add("inbag", -1);

    po::variables_map vm;

    try
    {
        po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
    }
    catch (po::invalid_command_line_syntax& e)
    {
        throw miniros::Exception(e.what());
    }
    catch (po::unknown_option& e)
    {
        throw miniros::Exception(e.what());
    }

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        exit(0);
    }

    if (vm.count("quiet"))
        opts.quiet = true;
    if (vm.count("plugin"))
        opts.plugin = vm["plugin"].as<std::string>();
    if (vm.count("param"))
        opts.param = vm["param"].as<std::string>();
    if (vm.count("bz2"))
        opts.compression = minibag::compression::BZ2;
    if (vm.count("lz4"))
        opts.compression = minibag::compression::LZ4;
    if (vm.count("inbag"))
        opts.inbag = vm["inbag"].as<std::string>();
    else
      throw miniros::Exception("You must specify bag to encrypt.");
    if (vm.count("outbag"))
        opts.outbag = vm["outbag"].as<std::string>();
    opts.buildOutbagName();

    return opts;
}

std::string getStringCompressionType(minibag::CompressionType compression)
{
    switch(compression)
    {
    case minibag::compression::Uncompressed: return "none";
    case minibag::compression::BZ2: return "bz2";
    case minibag::compression::LZ4: return "lz4";
    default: return "Unknown";
    }
}

int encrypt(EncryptorOptions const& options)
{
    if (!options.quiet)
    {
        std::cout << "Output bag:  " << options.outbag << "\n";
        std::cout << "Encryption:  " << options.plugin << ":" << options.param << "\n";
        std::cout << "Compression: " << getStringCompressionType(options.compression) << "\n";
    }
    minibag::Bag inbag(options.inbag, minibag::bagmode::Read);
    minibag::Bag outbag(options.outbag, minibag::bagmode::Write);
    // Compression type is per chunk, and cannot be retained.
    // If chunk-by-chunk encryption is implemented, compression type could be honored.
    outbag.setEncryptorPlugin(options.plugin, options.param);
    outbag.setCompression(options.compression);
    minibag::View view(inbag);
    std::unique_ptr<progress_display> progress;
    if (!options.quiet)
        progress.reset(new progress_display(view.size(), std::cout, "Progress:\n  ", "  ", "  "));
    for (minibag::View::const_iterator it = view.begin(); it != view.end(); ++it)
    {
        outbag.write(it->getTopic(), it->getTime(), *it, it->getConnectionHeader());
        if (progress)
            ++(*progress);
    }
    outbag.close();
    inbag.close();
    return 0;
}

/// Handles command `minibag encrypt ...`
int runEncrypt(int argc, char** argv)
{
    // Parse the command-line options
    EncryptorOptions opts;
    try
    {
        opts = parseEncryptOptions(argc, argv);
    }
    catch (miniros::Exception const& ex)
    {
        MINIROS_ERROR("Error reading options: %s", ex.what());
        return 1;
    }
    catch(std::regex_error const& ex)
    {
        MINIROS_ERROR("Error reading options: %s\n", ex.what());
        return 1;
    }

    return encrypt(opts);
}

int main(int argc, char** argv) {

  if (argc < 2) {
    std::cerr << "Expecting command name" << std::endl;
    return EXIT_FAILURE;
  }

  std::string command = argv[1];

  if (command == "play" || command == "record") {
    miniros::init(argc, argv, "minibag", miniros::init_options::AnonymousName);
  }

  if (command == "play")
    return runPlay(argc-1, argv+1);
  else if (command == "record")
    return runRecord(argc-1, argv+1);
  else if (command == "encrypt")
    return runEncrypt(argc-1, argv+1);
  return EXIT_FAILURE;
}
