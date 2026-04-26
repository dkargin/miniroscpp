//
// Created by dkargin on 8/2/25.
//

#ifndef MINIROS_HTTP_FS_ENDPOINT_H
#define MINIROS_HTTP_FS_ENDPOINT_H

#include <filesystem>

#include "../http_endpoint.h"
#include "../http_tools.h"

namespace miniros {
namespace http {

/// This endpoint serves static content from filesystem.
class MINIROS_DECL FilesystemEndpoint : public EndpointHandler {
public:
  /// @param uriRoot - URL prefix path.
  /// @param fsPath - path to root folder at filesystem.
  FilesystemEndpoint(const std::string& uriRoot, const std::string& fsPath);

  /// Get ContentType for specific file.
  virtual std::string contentTypeForFile(const std::filesystem::path& path) const;

  /// Enable custom HTML image viewer.
  void enableImageView(bool flag);

  Error handle(const network::ClientInfo& clientInfo, std::shared_ptr<HttpRequest> request) override;

protected:
  std::string prefix_path_;
  /// Path to a shared folder.
  std::filesystem::path root_path_;

  /// Show hidden files.
  bool show_hidden_ = false;

  /// Serve images through HTML page with navigation.
  bool image_view_enabled_ = false;
};

}
}
#endif //MINIROS_HTTP_FS_ENDPOINT_H
