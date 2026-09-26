//
// Created by dkargin on 8/19/25.
//

#include "master_endpoints.h"
#include "miniros_favicon.h"

#include "http/http_filters.h"
#include "http/http_request.h"
#include "http/http_server.h"
#include "miniros/http/http_printers.h"
#include "miniros/internal/json_tools.h"
#include "miniros/xmlrpcpp/XmlRpcValue.h"
#include "requester_info.h"
#include "node_ref.h"

#include "miniros/console.h"

#include <cstdlib>
#include <fstream>
#include <iterator>
#include <sstream>

namespace miniros {
namespace master {

using namespace http;

Error MasterRootEndpoint::handle(const network::ClientInfo& clientInfo, std::shared_ptr<http::HttpRequest> request)
{
  if (!internal)
    return Error::InternalError;

  const std::string title = "MiniROS master at " + internal->localHostname();
  std::string body = "<!doctype html><html><head><meta charset=\"utf-8\"/><title>" + title + "</title></head><body>";
  internal->renderMasterStatus(body, clientInfo.remoteAddress, clientInfo.localAddress);
  body += "</body></html>";
  request->setResponseBody(body, "text/html");
  request->setResponseStatusOk();
  return Error::Ok;
}

Error MasterFaviconEndpoint::handle(const network::ClientInfo& clientInfo, std::shared_ptr<http::HttpRequest> request)
{
  std::string_view vFavicon((const char*)(favicon), sizeof(favicon));
  request->setResponseBody(std::string(vFavicon), "image/x-icon");
  request->setResponseStatusOk();
  return Error::Ok;
}

Error MasterLogEndpoint::handle(const network::ClientInfo& clientInfo, std::shared_ptr<http::HttpRequest> request)
{
  if (!internal)
    return Error::InternalError;

  auto notConfigured = [&]() {
    request->setResponseStatus(404, "Not Found");
    request->setResponseBody(
      "<!doctype html><html><title>Log not configured</title><body>"
      "<p>log was not configured</p>"
      "<p><a href=\"/\">Back to master</a></p>"
      "</body></html>",
      "text/html");
    return Error::Ok;
  };

  const std::string path = internal->rosoutLogPath();
  if (path.empty() || !internal->rosoutLogConfigured())
    return notConfigured();

  std::ifstream in(path, std::ios::in | std::ios::binary);
  if (!in.is_open())
    return notConfigured();

  std::string body((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  request->setResponseHeader("Content-Disposition", "inline; filename=\"rosout.log\"");
  request->setResponseBody(body, "text/plain; charset=utf-8");
  request->setResponseStatusOk();
  return Error::Ok;
}

Error NodeInfoEndpoint::handle(const network::ClientInfo& clientInfo, std::shared_ptr<http::HttpRequest> request)
{
  std::string_view name = http::getNameFromUrlPath(request->path(), "/node/", true);

  std::string body = "<!doctype html><html><title>Mini ROS master</title><body>";

  if (name.empty() || !internal->renderNodeInfo(name, body, true)) {
    request->setResponseStatus(404, "Node not found");
  } else {
    request->setResponseStatusOk();
  }
  body += "</body></html>";
  request->setResponseBody(body, "text/html");
  return Error::Ok;
}

Error TopicInfoEndpoint::handle(const network::ClientInfo& clientInfo, std::shared_ptr<http::HttpRequest> request)
{
  std::string_view name = http::getNameFromUrlPath(request->path(), "/topic/", true);

  std::string body = "<!doctype html><html><title>Mini ROS master</title><body>";

  if (name.empty() || !internal->renderTopicInfo(name, body)) {
    request->setResponseStatus(404, "Topic not found");
  } else {
    request->setResponseStatusOk();
  }
  body += "</body></html>";
  request->setResponseBody(body, "text/html");
  return Error::Ok;
}

Error PublishedTopicsEndpoint::handle(const network::ClientInfo& clientInfo, std::shared_ptr<http::HttpRequest> request)
{
  if (!internal)
    return Error::InternalError;

  // Create RequesterInfo for the HTTP client
  RequesterInfo requesterInfo;
  requesterInfo.assign("http_client", clientInfo);

  // Get published topics
  auto topics = internal->handler.getPublishedTopics(requesterInfo, "");

  // Build JSON structure as a simple object: {"/topic1": "std_msgs/String", ...}
  using RpcValue = XmlRpc::XmlRpcValue;
  RpcValue jsonResponse = RpcValue::Dict();
  for (size_t i = 0; i < topics.size(); i++) {
    jsonResponse[topics[i][0]] = topics[i][1];  // topic name -> topic type
  }

  // Serialize to JSON
  std::ostringstream oss;
  miniros::JsonState state;
  jsonResponse.writeJson(oss, state, {});

  request->setResponseBody(oss.str(), "application/json");
  request->setResponseStatusOk();

  return Error::Ok;
}

Error TopicTypesEndpoint::handle(const network::ClientInfo& clientInfo, std::shared_ptr<http::HttpRequest> request)
{
  if (!internal)
    return Error::InternalError;

  // Get all topic types
  auto topicTypes = internal->regManager.getTopicTypes("http_client");

  // Build JSON structure as a simple object: {"/topic1": "std_msgs/String", ...}
  using RpcValue = XmlRpc::XmlRpcValue;
  RpcValue jsonResponse;
  for (const auto& [topicName, topicType] : topicTypes) {
    jsonResponse[topicName] = topicType;
  }

  // Serialize to JSON
  std::ostringstream oss;
  miniros::JsonState state;
  jsonResponse.writeJson(oss, state, {});
  request->setResponseStatusOk();
  request->setResponseBody(oss.str(), "application/json");

  return Error::Ok;
}

namespace {

bool parseFlag(const std::string& text, bool& out)
{
  if (text == "1" || text == "true" || text == "on") {
    out = true;
    return true;
  }
  if (text == "0" || text == "false" || text == "off") {
    out = false;
    return true;
  }
  return false;
}

std::string htmlEscape(const std::string& text)
{
  std::string out;
  out.reserve(text.size());
  for (char c : text) {
    switch (c) {
    case '&': out += "&amp;"; break;
    case '<': out += "&lt;"; break;
    case '>': out += "&gt;"; break;
    case '"': out += "&quot;"; break;
    default: out += c; break;
    }
  }
  return out;
}

/// Empty when the cache file can be used. Otherwise a short problem description.
std::string cacheFileProblem(const MasterCache::FileStatus& file)
{
  if (file.path.empty())
    return "Cache path is not known yet.";
  if (file.exists && file.readable && file.writable)
    return {};
  if (!file.exists && file.writable)
    return {};
  if (file.exists && file.readable)
    return "Permission denied: cache file is not writable.";
  if (file.exists)
    return "Permission denied: cache file cannot be read.";
  return "Cache directory is missing or not writable.";
}

std::string settingsForm(const Master::Internal& internal)
{
  const bool resolveOn = internal.resolver.resolveIp();
  const bool discoveryOn = internal.multimaster && internal.multimaster->discoveryEnabled();
  const bool persistenceOn = internal.cache.enabled();
  const int port = internal.discoveryPort();
  const int rpcPort = internal.rpcPort();
  const MasterCache::FileStatus cacheFile = internal.cache.fileStatus(rpcPort);
  const std::string cacheProblem = cacheFileProblem(cacheFile);

  std::ostringstream ss;
  ss << "<!doctype html><html><head><meta charset=\"utf-8\"/><title>MiniROS settings</title></head><body>";
  ss << "<h1>Local settings</h1>";
  ss << "<form method=\"GET\" action=\"/api2/settings\">";
  const char* resolveTip = "Rewrites XML-RPC node URIs to a known IP. Same flag as --resolve and /resolve_ip.";
  const char* discoveryTip = "Turning discovery off stops DISCOVER. Paired sync stays up.";
  const char* portTip = "UDP port for discovery and sync (--discovery). Changing it rebinds the socket.";
  ss << "<p><label title=\"" << htmlEscape(resolveTip) << "\">Resolve IP ";
  ss << "<select name=\"resolve\" title=\"" << htmlEscape(resolveTip) << "\">";
  ss << "<option value=\"1\"" << (resolveOn ? " selected" : "") << ">on</option>";
  ss << "<option value=\"0\"" << (resolveOn ? "" : " selected") << ">off</option>";
  ss << "</select></label></p>";
  ss << "<p><label title=\"" << htmlEscape(discoveryTip) << "\">Discovery ";
  ss << "<select name=\"discovery\" title=\"" << htmlEscape(discoveryTip) << "\">";
  ss << "<option value=\"1\"" << (discoveryOn ? " selected" : "") << ">on</option>";
  ss << "<option value=\"0\"" << (discoveryOn ? "" : " selected") << ">off</option>";
  ss << "</select></label></p>";
  ss << "<p><label title=\"" << htmlEscape(portTip) << "\">Discovery port ";
  ss << "<input name=\"discovery_port\" title=\"" << htmlEscape(portTip) << "\" value=\"" << port << "\"/>";
  ss << "</label></p>";
  ss << "<p><label>Persistence <select name=\"persistence\">";
  ss << "<option value=\"1\"" << (persistenceOn ? " selected" : "") << ">on</option>";
  ss << "<option value=\"0\"" << (persistenceOn ? "" : " selected") << ">off</option>";
  ss << "</select></label></p>";
  ss << "<p>Cache file: <code>" << htmlEscape(cacheFile.path.empty() ? "(unknown)" : cacheFile.path) << "</code>";
  if (!cacheProblem.empty())
    ss << "<br/><span style=\"color:#c62828;\">" << htmlEscape(cacheProblem) << "</span>";
  ss << "</p>";
  ss << "<p><button type=\"submit\">Save</button></p>";
  ss << "</form>";
  ss << "<p><a href=\"/\">Back to master</a></p>";
  ss << "</body></html>";
  return ss.str();
}

} // namespace

Error SettingsEndpoint::handle(const network::ClientInfo& clientInfo, std::shared_ptr<http::HttpRequest> request)
{
  (void)clientInfo;
  if (!internal)
    return Error::InternalError;

  if (htmlPage) {
    request->setResponseBody(settingsForm(*internal), "text/html");
    request->setResponseStatusOk();
    return Error::Ok;
  }

  const std::string resolveText = request->getParameter("resolve");
  const std::string discoveryText = request->getParameter("discovery");
  const std::string portText = request->getParameter("discovery_port");
  const std::string persistenceText = request->getParameter("persistence");

  bool resolve = false;
  bool discovery = false;
  bool persistence = false;
  int port = 0;
  if (!resolveText.empty() && !parseFlag(resolveText, resolve)) {
    request->setResponseStatus(400, "Bad Request");
    request->setResponseBody("", "application/json");
    return Error::Ok;
  }
  if (!discoveryText.empty() && !parseFlag(discoveryText, discovery)) {
    request->setResponseStatus(400, "Bad Request");
    request->setResponseBody("", "application/json");
    return Error::Ok;
  }
  if (!persistenceText.empty() && !parseFlag(persistenceText, persistence)) {
    request->setResponseStatus(400, "Bad Request");
    request->setResponseBody("", "application/json");
    return Error::Ok;
  }
  if (!portText.empty()) {
    char* end = nullptr;
    const long parsed = std::strtol(portText.c_str(), &end, 10);
    if (!end || *end != '\0' || parsed < 0 || parsed > 65535) {
      request->setResponseStatus(400, "Bad Request");
      request->setResponseBody("", "application/json");
      return Error::Ok;
    }
    port = static_cast<int>(parsed);
  }

  if (Error err = internal->applySettings(
        !resolveText.empty(), resolve,
        !discoveryText.empty(), discovery,
        !portText.empty(), port,
        !persistenceText.empty(), persistence); !err) {
    request->setResponseStatus(500, "Internal Server Error");
    request->setResponseBody("", "application/json");
    return Error::Ok;
  }

  auto accept = request->getHeader("Accept");
  if (accept.empty())
    accept = request->getHeader("accept");
  const bool wantsHtml = accept.find("text/html") != std::string::npos;
  const bool changed = !resolveText.empty() || !discoveryText.empty() || !portText.empty() || !persistenceText.empty();
  if (wantsHtml && changed) {
    request->setResponseStatusOk();
    request->setResponseBody(
      "<!doctype html><html><head><meta http-equiv=\"refresh\" content=\"0;url=/settings\"/></head>"
      "<body><p><a href=\"/settings\">Settings</a></p></body></html>",
      "text/html");
    return Error::Ok;
  }

  using RpcValue = XmlRpc::XmlRpcValue;
  RpcValue root;
  root["resolve"] = internal->resolver.resolveIp();
  root["discovery"] = internal->multimaster && internal->multimaster->discoveryEnabled();
  root["discovery_port"] = internal->discoveryPort();
  root["persistence"] = internal->cache.enabled();
  {
    const int rpcPort = internal->rpcPort();
    const MasterCache::FileStatus cacheFile = internal->cache.fileStatus(rpcPort);
    root["persistence_path"] = cacheFile.path;
    root["persistence_exists"] = cacheFile.exists;
    root["persistence_readable"] = cacheFile.readable;
    root["persistence_writable"] = cacheFile.writable;
  }
  std::ostringstream oss;
  miniros::JsonState state;
  root.writeJson(oss, state, {});
  request->setResponseBody(oss.str(), "application/json");
  request->setResponseStatusOk();
  return Error::Ok;
}

Error NodeUriEndpoint::handle(const network::ClientInfo& clientInfo, std::shared_ptr<http::HttpRequest> request)
{
  if (!internal)
    return Error::InternalError;

  auto fail = [&](int httpCode, const char* status) {
    request->setResponseStatus(httpCode, status);
    request->setResponseBody("", "application/json");
    return Error::Ok;
  };

  const std::string nodeName = request->getParameter("node");
  if (nodeName.empty())
    return fail(400, "Bad Request");

  // Empty `ip` keeps every known address. Otherwise a comma-separated family list
  // (`4`, `6`, `4,6`) selects which addresses to return in this one response.
  const std::string ip = request->getParameter("ip");
  bool filter = false;
  bool want4 = false;
  bool want6 = false;
  if (!ip.empty()) {
    filter = true;
    std::string token;
    for (size_t i = 0; i <= ip.size(); ++i) {
      if (i == ip.size() || ip[i] == ',') {
        if (token == "4" || token == "ip4" || token == "ipv4")
          want4 = true;
        else if (token == "6" || token == "ip6" || token == "ipv6")
          want6 = true;
        else
          return fail(400, "Bad Request");
        token.clear();
      } else if (ip[i] != ' ') {
        token.push_back(ip[i]);
      }
    }
  }

  const std::shared_ptr<NodeRef> node = internal->regManager.getNodeByName(nodeName);
  if (!node)
    return fail(404, "Not Found");

  using RpcValue = XmlRpc::XmlRpcValue;
  RpcValue addresses = RpcValue::Array(0);
  int n = 0;
  for (const network::NetAddress& addr : internal->resolver.listNodeAddresses(node)) {
    if (filter) {
      if (addr.type() == network::NetAddress::AddressIPv4) {
        if (!want4)
          continue;
      } else if (addr.type() == network::NetAddress::AddressIPv6) {
        if (!want6)
          continue;
      } else {
        continue;
      }
    }
    addresses[n++] = addr.address;
  }
  if (n == 0)
    return fail(404, "Not Found");

  std::ostringstream oss;
  miniros::JsonState state;
  addresses.writeJson(oss, state, {});
  request->setResponseBody(oss.str(), "application/json");
  request->setResponseStatusOk();
  return Error::Ok;
}

Error MultimasterApiEndpoint::handle(const network::ClientInfo& clientInfo, std::shared_ptr<http::HttpRequest> request)
{
  if (!internal)
    return Error::InternalError;

  constexpr std::string_view kPrefix = "/api2/multimaster/";
  std::string_view cmd = http::getNameFromUrlPath(request->path(), kPrefix, false);
  if (cmd.empty() && (request->path() == "/api2/multimaster" || request->path() == "/api2/multimaster/"))
    cmd = "status";

  using RpcValue = XmlRpc::XmlRpcValue;

  auto accept = request->getHeader("Accept");
  if (accept.empty())
    accept = request->getHeader("accept");
  const bool formatJson = request->getParameter("format") == "json";
  const bool wantsHtml = !formatJson &&
    (accept.find("text/html") != std::string::npos) &&
    !(accept.find("application/json") != std::string::npos &&
      accept.find("text/html") > accept.find("application/json"));

  auto replyJson = [&](const RpcValue& body, int httpCode = 200, const char* status = "OK") {
    std::ostringstream oss;
    miniros::JsonState state;
    body.writeJson(oss, state, {});
    request->setResponseBody(oss.str(), "application/json");
    request->setResponseStatus(httpCode, status);
    return Error::Ok;
  };

  auto replyCommand = [&](bool ok, const std::string& message, int httpCode, const char* status) {
    if (wantsHtml) {
      std::ostringstream ss;
      ss << "<!doctype html><html><title>Mini ROS master</title><body>";
      ss << "<h1>Multimaster</h1>";
      ss << "<p style=\"color:" << (ok ? "#2e7d32" : "#c62828") << ";font-weight:bold;\">";
      ss << (ok ? "OK: " : "Error: ") << message << "</p>";
      ss << "<p>" << print::Url("/", "Back to master") << "</p>";
      if (ok)
        ss << "<meta http-equiv=\"refresh\" content=\"2;url=/\">";
      ss << "</body></html>";
      request->setResponseBody(ss.str(), "text/html");
      request->setResponseStatus(httpCode, status);
      return Error::Ok;
    }
    RpcValue root;
    root["ok"] = ok;
    if (ok)
      root["message"] = message;
    else
      root["error"] = message;
    return replyJson(root, httpCode, status);
  };

  if (!internal->multimaster) {
    return replyCommand(false, "multimaster unavailable", 500, "Internal Server Error");
  }

  if (cmd.empty() || cmd == "status" || cmd == "help") {
    RpcValue root;
    root["ok"] = true;
    root["guid"] = internal->uuid.toString();
    root["udp_port"] = internal->multimaster->udpPort();
    root["multicast"] = internal->multimaster->multicastEndpoint();
    root["multicast_error"] = internal->multimaster->multicastError();
    root["token_set"] = internal->multimaster->hasToken();

    RpcValue peers = RpcValue::Array(0);
    int paired = 0;
    int i = 0;
    for (const PeerInfo& peer : internal->multimaster->listPeers()) {
      RpcValue p;
      p["uuid"] = peer.uuid.toString();
      p["state"] = MultimasterManager::peerStateName(peer.state);
      p["uri"] = peer.httpUri().str();
      p["hostname"] = peer.hostname;
      p["label"] = peer.displayLabel();
      p["address"] = peer.lastAddress.valid() ? peer.lastAddress.str() : std::string();
      p["pairable"] = peer.state != PeerState::GuidCollision &&
                      peer.state != PeerState::Paired &&
                      peer.state != PeerState::Requesting;
      p["remote_token"] = peer.remoteHasToken;
      p["token_match"] = peer.tokenMatch;
      p["pubs"] = static_cast<int>(peer.foreignPubs);
      p["subs"] = static_cast<int>(peer.foreignSubs);
      p["srvs"] = static_cast<int>(peer.foreignSrvs);
      if (peer.state == PeerState::Paired)
        ++paired;
      peers[i++] = p;
    }
    root["paired_count"] = paired;
    root["peers"] = peers;

    if (cmd == "help") {
      RpcValue cmds = RpcValue::Array(0);
      cmds[0] = "status";
      cmds[1] = "connect?uuid=...&token=...";
      cmds[2] = "disconnect";
      root["commands"] = cmds;
    }
    return replyJson(root);
  }

  if (cmd == "connect") {
    std::string nodeName = request->getParameter("node");
    std::string uuidStr = request->getParameter("uuid");
    std::string token = request->getParameter("token");

    Error err = Error::InvalidValue;

    if (!nodeName.empty()) {
      if (auto node = internal->regManager.getNodeByName(nodeName)) {
        if (node->getNodeFlags() & NodeRef::NODE_LOCAL) {
          return replyCommand(false, "cannot pair with this master (local /miniroscore)",
            409, "Conflict");
        }
      }
    }

    PeerInfo matchedPeer;
    bool foundPeer = false;
    if (!uuidStr.empty()) {
      for (const PeerInfo& peer : internal->multimaster->listPeers()) {
        if (peer.uuid.toString() == uuidStr) {
          matchedPeer = peer;
          foundPeer = true;
          break;
        }
      }
      if (!foundPeer) {
        return replyCommand(false, "unknown uuid", 404, "Not Found");
      }
      if (matchedPeer.state == PeerState::GuidCollision || uuidStr == internal->uuid.toString()) {
        return replyCommand(false, "cannot pair: remote master uses this master's GUID",
          409, "Conflict");
      }
    }

    if (token.empty() && !internal->multimaster->hasToken() &&
        foundPeer && matchedPeer.remoteHasToken && !matchedPeer.tokenMatch) {
      return replyCommand(false, "token required to join remote mesh", 400, "Bad Request");
    }

    if (foundPeer) {
      err = internal->multimaster->requestPair(matchedPeer.uuid, token);
    } else if (!nodeName.empty()) {
      err = internal->multimaster->requestPairByNodeName(nodeName, token);
    } else {
      return replyCommand(false, "missing uuid or node", 400, "Bad Request");
    }

    if (err)
      return replyCommand(true, "pair request sent", 200, "OK");
    if (err.code == Error::PermissionDenied)
      return replyCommand(false, "cannot pair with this master", 409, "Conflict");
    return replyCommand(false, err.toString(), 400, "Bad Request");
  }

  if (cmd == "disconnect") {
    Error err = internal->multimaster->disconnectAll();
    if (err)
      return replyCommand(true, "disconnected", 200, "OK");
    return replyCommand(false, err.toString(), 500, "Internal Server Error");
  }

  return replyCommand(false, std::string("unknown command: ") + std::string(cmd), 404, "Not Found");
}

Error DebugApiEndpoint::handle(const network::ClientInfo& clientInfo, std::shared_ptr<http::HttpRequest> request)
{
  if (!internal)
    return Error::InternalError;

  constexpr std::string_view kPrefix = "/debugAPI/";
  std::string_view cmd = http::getNameFromUrlPath(request->path(), kPrefix, false);
  // Also accept exact "/debugAPI" as help.
  if (cmd.empty() && (request->path() == "/debugAPI" || request->path() == "/debugAPI/"))
    cmd = "help";

  std::stringstream ss;
  ss << "<!doctype html><html><title>Mini ROS master</title><body>";

  if (cmd.empty() || cmd == "help") {
    ss << "<h1>Debug API</h1>";
    ss << "<ul>";
    ss << "<li><a href=\"/debugAPI/shutdown\">/debugAPI/shutdown</a> - request master exit</li>";
    ss << "</ul>";
    ss << "<p>" << print::Url("/", "BACK") << "</p>";
    ss << "</body></html>";
    request->setResponseBody(ss.str(), "text/html");
    request->setResponseStatusOk();
    return Error::Ok;
  }

  if (cmd == "shutdown") {
    internal->shutdownRequested.store(true);
    MINIROS_WARN("Debug API: shutdown requested via HTTP");
    ss << "<h1>Debug Shutdown</h1>";
    ss << "<p>Master shutdown requested.</p>";
    ss << "</body></html>";
    request->setResponseBody(ss.str(), "text/html");
    request->setResponseStatusOk();
    return Error::Ok;
  }

  ss << "<h1>Debug API</h1>";
  ss << "<p>Unknown command: " << std::string(cmd) << "</p>";
  ss << "<p><a href=\"/debugAPI/\">help</a></p>";
  ss << "</body></html>";
  request->setResponseBody(ss.str(), "text/html");
  request->setResponseStatus(404, "Not Found");
  return Error::Ok;
}

}
}
