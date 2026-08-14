//
// Created by dkargin on 8/25/25.
//

#include "master_internal.h"

#include "miniros/network/host_info.h"
#include "miniros/http/http_printers.h"
#include "miniros/rostime.h"
#include "node_ref.h"

#include <set>
#include <sstream>
#include <vector>

namespace miniros {
namespace master {

using namespace http;

namespace {

std::string formatUptime(const WallDuration& d)
{
  int64_t sec = d.sec;
  if (sec < 0)
    sec = 0;
  const int64_t days = sec / 86400;
  sec %= 86400;
  const int hours = static_cast<int>(sec / 3600);
  sec %= 3600;
  const int mins = static_cast<int>(sec / 60);
  const int secs = static_cast<int>(sec % 60);
  std::ostringstream os;
  if (days > 0)
    os << days << "d ";
  os << hours << "h " << mins << "m " << secs << "s";
  return os.str();
}

}

void Master::Internal::renderMasterStatus(std::string& output) const
{
  std::stringstream ss;
  const std::string host = localHostname();
  ss << "<h1>MiniROS master at " << host << "</h1>\n";
  ss << "<p>GUID: <code>" << uuid.toString() << "</code>";
  ss << " | uptime: " << formatUptime(SteadyTime::now() - startTime);
  ss << " | " << print::Url("/log", "log");
  if (!rosoutLogConfigured())
    ss << " <em>(not configured)</em>";
  ss << "</p>\n";

  if (multimaster) {
    const bool localHasToken = multimaster->hasToken();
    ss << "<details>\n<summary>Config</summary>\n";
    ss << "<p>UDP sync port: " << multimaster->udpPort();
    ss << " | discovery: on";
    const std::string mc = multimaster->multicastEndpoint();
    ss << " | multicast: " << (mc.empty() ? "off" : mc);
    const std::string mcErr = multimaster->multicastError();
    if (mc.empty() && !mcErr.empty())
      ss << " <small>(" << mcErr << "; LAN discovery uses UDP broadcast on port "
         << multimaster->udpPort() << ")</small>";
    ss << " | token: " << (localHasToken ? "set" : "none") << "</p>\n";
    ss << "<p><small>Token is required only when joining a mesh that already has one. "
          "Open masters (no token) can pair without entering a token.</small></p>\n";
    ss << "</details>\n";
  }

  ss << print::HB("Nodes:");
  ss << "<ul>";
  for (const std::shared_ptr<NodeRef>& r : regManager.listAllNodes()) {
    const int flags = r->getNodeFlags();
    // Peer masters belong in Discovery, not the ROS1 node graph.
    if ((flags & NodeRef::NODE_MASTER) && !(flags & NodeRef::NODE_LOCAL))
      continue;
    const std::string& name = r->id();
    std::string url = r->getApi();
    ss << "<li>";
    ss << print::PrefixUrl("node", name, name) << ": " << print::Url(url, url);
    if (flags & NodeRef::NODE_LOCAL)
      ss << " <em>It's me</em>";
    else if (flags & NodeRef::NODE_FOREIGN)
      ss << " <em>[foreign]</em>";
    ss << "</li>";
  }
  ss << "</ul>\n";

  ss << print::HB("Topics:");
  ss << "<ul>";
  for (const auto& [topic, type] : regManager.getTopicTypes("root")) {
    ss << "<li>";
    ss << print::PrefixUrl("topic", topic, topic) << " (" << type << ")";
    ss << "</li>";
  }
  ss << "</ul>\n";

  ss << print::HB("Peers:");
  ss << "<ul>";

  for (const std::shared_ptr<network::HostInfo> host : resolver.getHosts()) {
    ss << "<li><div><dl>";
    ss << "<dt>" << host->hostname << "</dt>";
    host->iterate([&ss](const network::NetAddress& address) {
      // Skip boring addresses.
            if (address.isLoopback())
              return;
            if (address.isUnspecified())
              return;
            ss << "<dd>" << address.str() << "</dd>" << std::endl;
    });

    ss << "</dl></div></li>";
  }
  ss << "</ul>\n";

  if (multimaster) {
    const bool localHasToken = multimaster->hasToken();
    ss << print::HB("Discovery:");

    if (multimaster->hasPairedPeers()) {
      ss << "<form method=\"GET\" action=\"/api2/multimaster/disconnect\" style=\"margin:0.5em 0;\">";
      ss << "<button type=\"submit\">Disconnect all</button>";
      ss << "</form>\n";
    }

    const auto peers = multimaster->listPeers();
    bool hasGuidCollision = false;
    for (const PeerInfo& peer : peers) {
      if (peer.state == PeerState::GuidCollision) {
        hasGuidCollision = true;
        break;
      }
    }
    if (hasGuidCollision) {
      ss << "<p style=\"color:#c62828;font-weight:bold;\">Warning: another master is advertising this GUID "
            "(copied cache or cloned identity). It is listed below in red and cannot be paired. "
            "Delete <code>cache.&lt;port&gt;</code> on that host or start with <code>--no-cache</code>.</p>\n";
    }

    if (peers.empty()) {
      ss << "<p><em>No peer masters discovered yet.</em></p>\n";
    } else {
      ss << "<ul>";
      for (const PeerInfo& peer : peers) {
        const bool collision = peer.state == PeerState::GuidCollision;
        if (collision)
          ss << "<li style=\"color:#c62828;\">";
        else
          ss << "<li>";

        const std::string uri = peer.masterUri.empty() ? std::string() : peer.masterUri.str();
        if (!uri.empty()) {
          ss << print::Url(uri, uri);
          if (peer.lastAddress.valid() && peer.masterUri.host != peer.lastAddress.address)
            ss << " (" << peer.lastAddress.address << ")";
        } else if (peer.lastAddress.valid()) {
          ss << peer.lastAddress.str();
        } else {
          ss << "<em>(no URI yet)</em>";
        }

        ss << " <b>" << MultimasterManager::peerStateName(peer.state) << "</b>";

        if (collision) {
          ss << " (pairing forbidden)";
        } else if (peer.state != PeerState::Paired && peer.state != PeerState::Requesting) {
          const bool tokenRequired = peer.remoteHasToken && !peer.tokenMatch;
          ss << "<form method=\"GET\" action=\"/api2/multimaster/connect\" style=\"display:inline;margin-left:0.5em;\">";
          ss << "<input type=\"hidden\" name=\"uuid\" value=\"" << peer.uuid.toString() << "\"/>";
          ss << "<input type=\"password\" name=\"token\" placeholder=\"";
          if (tokenRequired)
            ss << "collective token (required)";
          else if (localHasToken)
            ss << "token (optional, already set)";
          else
            ss << "token (optional)";
          ss << "\"";
          if (tokenRequired)
            ss << " required";
          ss << "/>";
          ss << "<button type=\"submit\">pair</button>";
          ss << "</form>";
        }

        ss << "<details style=\"margin:0.25em 0 0.5em 1em;\">";
        ss << "<summary>details</summary>";
        ss << "<p>GUID: <code>" << peer.uuid.toString() << "</code></p>";
        if (peer.lastAddress.valid())
          ss << "<p>from " << peer.lastAddress.str() << "</p>";
        ss << "<p>remote_token=" << (peer.remoteHasToken ? "yes" : "no");
        ss << ", match=" << (peer.tokenMatch ? "yes" : "no") << "</p>";
        ss << "<p>pubs=" << peer.foreignPubs << " subs=" << peer.foreignSubs
           << " srvs=" << peer.foreignSrvs << "</p>";
        ss << "</details>";
        ss << "</li>";
      }
      ss << "</ul>\n";
    }
  }

  output += ss.str();
}

Error Master::Internal::renderTopicInfo(const std::string_view& name, std::string& output) const
{
  std::string type = regManager.getTopicType(name);
  const bool hasType = !type.empty();

  std::vector<std::shared_ptr<NodeRef>> publishers;
  size_t numPubs = regManager.iteratePublishers(name, [&](const std::shared_ptr<NodeRef>& node) {
    publishers.push_back(node);
    return true;
  });

  std::vector<std::shared_ptr<NodeRef>> subscribers;
  size_t numSubs = regManager.iterateSubscribers(name, [&](const std::shared_ptr<NodeRef>& node) {
    subscribers.push_back(node);
    return true;
  });

  if (!hasType && numPubs == 0 && numSubs == 0) {
    return Error::FileNotFound;
  }

  if (!hasType)
    type = "unknown";

  std::stringstream ss;
  ss << "<p>" << print::HB(name) << " ";
  ss << "(" << type << ")";
  ss << "</p>" << std::endl;

  // Publishers:
  //  - node1
  //  - node2
  ss << "<p>" << print::HB("Publishers: ") << std::endl;
  ss << "<ul>";
  for (const std::shared_ptr<NodeRef>& node: publishers) {
    const std::string& name = node->id();
    std::string url = node->getApi();
    ss << "<li>";
    ss << print::PrefixUrl("/node", name, name) << ": " << print::Url(url, url);
    ss << "</li>";
  }
  ss << "</ul>";
  if (!numPubs) {
    ss << "none" << std::endl;
  }
  ss << "</p>" << std::endl;

  // Subscribers:
  //  - node1
  //  - node2
  ss << "<p>" << print::HB("Subscribers: ") << std::endl;
  ss << "<ul>";
  for (const std::shared_ptr<NodeRef>& node: subscribers) {
    const std::string& name = node->id();
    std::string url = node->getApi();
    ss << "<li>";
    ss << print::PrefixUrl("/node", name, name) << ": " << print::Url(url, url);
    ss << "</li>";
  }
  ss << "</ul>";
  if (!numSubs) {
    ss << "none" << std::endl;
  }
  ss << "</p>" << std::endl;

  ss << "<p>" << print::Url("/", "BACK") << "</p>" << std::endl;

  output += ss.str();
  return Error::Ok;
}

Error Master::Internal::renderNodeInfo(const std::string_view& name, std::string& output, bool showInternalInfo) const
{
  std::shared_ptr<NodeRef> nodePtr = regManager.getNodeByName(name);

  if (!nodePtr)
    return Error::FileNotFound;

  std::stringstream ss;

  // Render:
  // 1. Basic:
  //    - name, API link,
  //    - known IP addresses. href to machine page.
  //    - last contact time.

  ss << "<p>" << print::HB(name) << std::endl;
  const std::string url = nodePtr->getUrl().str();
  ss << "URL: " << print::Url(url, url);

  // Display flags
  int flags = nodePtr->getNodeFlags();
  ss << "<br/>Flags: ";
  std::vector<std::string> flagNames;
  if (flags & NodeRef::NODE_LOCAL) flagNames.push_back("LOCAL");
  if (flags & NodeRef::NODE_FOREIGN) flagNames.push_back("FOREIGN");
  if (flags & NodeRef::NODE_MINIROS) flagNames.push_back("MINIROS");
  if (flags & NodeRef::NODE_MASTER) flagNames.push_back("MASTER");
  if (flagNames.empty()) {
    ss << "none";
  } else {
    for (size_t i = 0; i < flagNames.size(); ++i) {
      if (i > 0) ss << ", ";
      ss << flagNames[i];
    }
  }

  // Pair form only for remote peer masters — not this process's /miniroscore.
  if ((flags & NodeRef::NODE_MASTER) && !(flags & NodeRef::NODE_LOCAL)) {
    const bool localHasToken = multimaster && multimaster->hasToken();
    bool remoteHasToken = false;
    bool tokenMatch = false;
    bool guidCollision = false;
    if (multimaster) {
      for (const PeerInfo& peer : multimaster->listPeers()) {
        if (peer.masterUri.str() == url) {
          remoteHasToken = peer.remoteHasToken;
          tokenMatch = peer.tokenMatch;
          guidCollision = peer.state == PeerState::GuidCollision;
          break;
        }
      }
    }
    if (guidCollision) {
      ss << "<br/><span style=\"color:#c62828;font-weight:bold;\">Cannot pair: this peer uses this master's GUID.</span>";
    } else {
      const bool tokenRequired = remoteHasToken && !tokenMatch;
      ss << "<br/><form method=\"GET\" action=\"/api2/multimaster/connect\">";
      ss << "<input type=\"hidden\" name=\"node\" value=\"" << std::string(name) << "\"/>";
      ss << "<input type=\"password\" name=\"token\" placeholder=\"";
      if (tokenRequired)
        ss << "collective token (required)";
      else if (localHasToken)
        ss << "token (optional, already set)";
      else
        ss << "token (optional)";
      ss << "\"";
      if (tokenRequired)
        ss << " required";
      ss << "/>";
      ss << "<button type=\"submit\">pair</button>";
      ss << "</form>";
    }
  }

  ss << "</p>";

  if (showInternalInfo) {
    ss << "<p>State = " << nodePtr->getState().toString() << "</p>";
    ss << "<p>Requests = " << nodePtr->getQueuedRequests() << "</p>";
  }

  // Do not hold NodeRef and RegistrationManager together: register/GC take
  // RegistrationManager then NodeRef. Nested locks here inverted that order
  // and deadlocked the HTTP callback against Master::update().
  std::set<std::string> subscriptions, publications, services;
  {
    NodeRef::Lock nodeLock(*nodePtr);
    subscriptions = nodePtr->getSubscriptionsLocked(nodeLock);
    publications = nodePtr->getPublicationsLocked(nodeLock);
    services = nodePtr->getServicesLocked(nodeLock);
  }
  const auto topicTypes = regManager.getTopicTypes("root");

  // 2. Subscriptions: a list of subscribed topics with types and hrefs
  ss << "<p>" << print::HB("Subscriptions: ") << std::endl;
  if (!subscriptions.empty()) {
    ss << "<ul>";
    for (const std::string& topic: subscriptions) {
      auto it = topicTypes.find(topic);
      ss << "<li>" << print::PrefixUrl("/topic", topic, topic);
      if (it != topicTypes.end()) {
        ss << " (" << it->second << ")";
      }
      ss << "</li>";
    }
    ss << "</ul>" << std::endl;
  } else {
    ss << "none" << std::endl;
  }
  ss << "</p>" << std::endl;

  // 3. Publications: a list of published topics with hrefs.
  ss << "<p>" << print::HB("Publications: ") << std::endl;
  if (!publications.empty()) {
    ss << "<ul>";
    for (const std::string& topic: publications) {
      auto it = topicTypes.find(topic);
      ss << "<li>" << print::PrefixUrl("/topic", topic, topic);
      if (it != topicTypes.end()) {
        ss << " (" << it->second << ")";
      }
      ss << "</li>";
    }
    ss << "</ul>" << std::endl;
  } else {
    ss << "none" << std::endl;
  }
  ss << "</p>" << std::endl;

  // 4. Services.
  ss << "<p>" << print::HB("Services: ") << std::endl;
  if (!services.empty()) {
    ss << "<ul>";
    for (const std::string& service: services) {
      ss << "<li>" << service << "</li>";
    }
    ss << "</ul>";
  } else {
    ss << "none";
  }
  ss << "</p>" << std::endl;

  ss << "<p>" << print::Url("/", "BACK") << "</p>" << std::endl;

  output += ss.str();
  return Error::Ok;
}

} // namespace master

} // namespace miniros
