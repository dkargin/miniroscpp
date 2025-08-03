//
// Created by dkargin on 8/2/25.
//

#include <cstdio>
#include <filesystem>

#include "miniros/http/endpoints/filesystem.h"

namespace miniros {
namespace http {

FilesystemEndpoint::FilesystemEndpoint(const std::string& uriRoot, const std::string& fsPath)
  :prefix_path_(uriRoot),  root_path_(fsPath)
{}

Error FilesystemEndpoint::handle(const HttpFrame& frame, const network::ClientInfo& clientInfo,
  HttpResponseHeader& responseHeader, std::string& body)
{
  // TODO:
  // 1. Read path.
  // 2. Open and read file.
  // 3. Deliver.
  return Error::NotImplemented;
}


}
}