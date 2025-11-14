//
// Created by dkargin on 8/2/25.
//

#include <cstdio>
#include <filesystem>
#include <fstream>

#include "miniros/http/endpoints/filesystem.h"
#include "miniros/http/http_filters.h"
#include "miniros/http/http_printers.h"

namespace miniros {
namespace http {

FilesystemEndpoint::FilesystemEndpoint(const std::string& uriRoot, const std::string& fsPath)
  :prefix_path_(uriRoot),  root_path_(fsPath)
{}

bool isSubpath(const std::filesystem::path& path, const std::filesystem::path& base) {
  const auto mismatch_pair = std::mismatch(path.begin(), path.end(), base.begin(), base.end());
  return mismatch_pair.second == base.end();
}

Error FilesystemEndpoint::handle(const HttpParserFrame& frame, const network::ClientInfo& clientInfo,
  HttpResponseHeader& responseHeader, std::string& body)
{
  // 1. Read path.

  std::string_view requestPath = frame.getPath();
  std::string_view path = getNameFromUrlPath(requestPath, prefix_path_, false);

  /*
  if (path.empty()) {
    return Error::InvalidAddress;
  }*/

  std::filesystem::path fsPath = std::filesystem::absolute(root_path_ / path);
  if (!std::filesystem::exists(fsPath)) {
    return Error::FileNotFound;
  }

  std::filesystem::path fullRootPath = std::filesystem::absolute(root_path_);
  /// Check if we are still within `root_path_`
  if (!isSubpath(fsPath, fullRootPath)) {
    MINIROS_WARN("Client tried to access path \"%s\" out of prefix path", fsPath.c_str());
    return Error::FileNotFound;
  }

  if (std::filesystem::is_directory(fsPath)) {
    responseHeader.contentType = "text/html";
    responseHeader.statusCode = 200;
    responseHeader.status = "OK";

    std::vector<std::filesystem::path> directories;
    std::vector<std::filesystem::path> files;

    for (const std::filesystem::path& dirEntry : std::filesystem::directory_iterator(fsPath)) {
      std::filesystem::path entry = std::filesystem::relative(dirEntry, fsPath);
      if (!show_hidden_ && !entry.empty()) {
        const std::string& str = entry.string();
        if (str[0] == '.')
          continue;
      }
      if (std::filesystem::is_directory(dirEntry)) {
        directories.push_back(entry);
      } else {
        files.push_back(entry);
      }
    }

    std::sort(directories.begin(), directories.end());
    std::sort(files.begin(), files.end());

    std::stringstream ss;
    ss << "<!doctype html><html><title>Index of " << fsPath << "</title><body>";
    ss << "<h1>Index of " << fsPath << "</h1><hr/>";

    if (!path.empty()) {
      std::filesystem::path p = std::filesystem::path(path).parent_path();
      ss << "<p>" << print::PrefixUrl(prefix_path_, p.string(), "../") << "</p>" << std::endl;
    }

    for (const std::filesystem::path& dir : directories) {
      std::filesystem::path p = std::filesystem::path(path) / dir;
      ss << print::PrefixUrl(prefix_path_, p.string(), dir.string() + "/") << "<br/>" << std::endl;
    }

    for (const std::filesystem::path& file : files) {
      std::filesystem::path p = std::filesystem::path(path) / file;
      ss << print::PrefixUrl(prefix_path_, p.string(), file.string()) << "<br/>" << std::endl;
    }

    ss << "<hr/>" << std::endl;
    ss << "</body></html>";

    body = ss.str();
  } else {
    // 2. Open and read file.
    std::ifstream in(fsPath);

    if (!in.is_open()) {
      return Error::FileNotFound;
    }

    std::istreambuf_iterator<char> it{in}, end;
    body.assign(it, end);
  }
  // 3. Deliver.
  return Error::Ok;
}


}
}