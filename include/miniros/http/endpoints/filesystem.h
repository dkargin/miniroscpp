//
// Created by dkargin on 8/2/25.
//

#ifndef MINIROS_HTTP_FS_ENDPOINT_H
#define MINIROS_HTTP_FS_ENDPOINT_H

#include "../http_endpoint.h"
#include "../http_tools.h"

namespace miniros {
namespace http {

/// This endpoint serves static content from filesystem.
class FilesystemEndpoint : public EndpointHandler {
public:
  /// @param uriRoot - URL prefix path.
  /// @param fsPath - path to root folder at filesystem.
  FilesystemEndpoint(const std::string& uriRoot, const std::string& fsPath);

  Error handle(const HttpFrame& frame, const network::ClientInfo& clientInfo,
    HttpResponseHeader& responseHeader, std::string& body) override;

protected:
  std::string prefix_path_;
  /// Path to a shared folder.
  std::string root_path_;
};

}
}
#endif //MINIROS_HTTP_FS_ENDPOINT_H
