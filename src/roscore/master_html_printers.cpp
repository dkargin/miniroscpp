//
// Created by dkargin on 8/25/25.
//

#include "master_internal.h"

#include "miniros/network/host_info.h"
#include "miniros/http/http_printers.h"
#include "node_ref.h"

#include <vector>

namespace miniros {
namespace master {

using namespace http;

void Master::Internal::renderMasterStatus(std::string& output) const
{
  std::stringstream ss;
  ss << print::HB("Nodes:");
  ss << "<ul>";
  for (const std::shared_ptr<NodeRef>& r : regManager.listAllNodes()) {
    const std::string& name = r->id();
    std::string url = r->getApi();
    ss << "<li>";
    ss << print::PrefixUrl("node", name, name) << ": " << print::Url(url, url);
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

  output += ss.str();
}

Error Master::Internal::renderTopicInfo(const std::string_view& name, std::string& output) const
{
  std::stringstream ss;

  std::string type = regManager.getTopicType(name);
  ss << "<p>" << print::HB(name) << " ";

  if (type.empty())
    type = "unknown";
  ss << "(" << type << ")";
  ss << "</p>" << std::endl;

  // Publishers:
  //  - node1
  //  - node2
  ss << "<p>" << print::HB("Publishers: ") << std::endl;
  ss << "<ul>";
  size_t numPubs = regManager.iteratePublishers(name, [&](const std::shared_ptr<NodeRef>& node) {
    const std::string& name = node->id();
    std::string url = node->getApi();
    ss << "<li>";
    ss << print::PrefixUrl("/node", name, name) << ": " << print::Url(url, url);
    ss << "</li>";
    return true;
  });
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
  size_t numSubs = regManager.iterateSubscribers(name, [&](const std::shared_ptr<NodeRef>& node) {
    const std::string& name = node->id();
    std::string url = node->getApi();
    ss << "<li>";
    ss << print::PrefixUrl("/node", name, name) << ": " << print::Url(url, url);
    ss << "</li>";
    return true;
  });
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

  // Add "pair" button if node has NODE_MASTER flag
  if (flags & NodeRef::NODE_MASTER) {
    std::string pairUrl = "/api2/multimaster/connect?node=" + std::string(name);
    ss << "<br/><form method=\"GET\" action=\"" << pairUrl << "\">";
    ss << "<button type=\"submit\">pair</button>";
    ss << "</form>";
  }

  ss << "</p>";

  if (showInternalInfo) {
    ss << "<p>State = " << nodePtr->getState().toString() << "</p>";
    ss << "<p>Requests = " << nodePtr->getQueuedRequests() << "</p>";
  }


  std::unique_lock nodeLock(*nodePtr);

  std::unique_lock<const RegistrationManager> regLock(regManager);

  const auto& topicTypes = regManager.getTopicTypesUnsafe(regLock);

  // 2. Subscriptions: a list of subscribed topics with types and hrefs
  ss << "<p>" << print::HB("Subscriptions: ") << std::endl;
  if (const auto& subscriptions = nodePtr->getSubscriptionsUnsafe(); !subscriptions.empty()) {
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
  if (const auto& publications = nodePtr->getPublicationsUnsafe(); !publications.empty()) {
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
  if (const auto& services = nodePtr->getServicesUnsafe(); !services.empty()) {
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
