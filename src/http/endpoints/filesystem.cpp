//
// Created by dkargin on 8/2/25.
//

#include <cstdio>
#include <cctype>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <unordered_map>

#include "miniros/http/endpoints/filesystem.h"
#include "miniros/http/http_filters.h"
#include "miniros/http/http_printers.h"
#include "miniros/http/http_request.h"

namespace miniros {
namespace http {

FilesystemEndpoint::FilesystemEndpoint(const std::string& uriRoot, const std::string& fsPath)
  :prefix_path_(uriRoot),  root_path_(fsPath)
{}

bool isSubpath(const std::filesystem::path& path, const std::filesystem::path& base) {
  const auto mismatch_pair = std::mismatch(path.begin(), path.end(), base.begin(), base.end());
  return mismatch_pair.second == base.end();
}

std::string FilesystemEndpoint::contentTypeForFile(const std::filesystem::path& path) const
{
  std::string ext = path.extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });

  static const std::unordered_map<std::string, std::string> kMimeByExt{
    {".html", "text/html"},
    {".htm", "text/html"},
    {".css", "text/css"},
    {".js", "application/javascript"},
    {".mjs", "application/javascript"},
    {".json", "application/json"},
    {".xml", "application/xml"},
    {".txt", "text/plain"},
    {".csv", "text/csv"},
    {".md", "text/markdown"},
    {".pdf", "application/pdf"},
    {".zip", "application/zip"},
    {".gz", "application/gzip"},
    {".tar", "application/x-tar"},
    {".rar", "application/vnd.rar"},
    {".7z", "application/x-7z-compressed"},
    {".jpg", "image/jpeg"},
    {".jpeg", "image/jpeg"},
    {".png", "image/png"},
    {".gif", "image/gif"},
    {".bmp", "image/bmp"},
    {".webp", "image/webp"},
    {".svg", "image/svg+xml"},
    {".ico", "image/x-icon"},
    {".mp3", "audio/mpeg"},
    {".wav", "audio/wav"},
    {".ogg", "audio/ogg"},
    {".mp4", "video/mp4"},
    {".webm", "video/webm"},
    {".avi", "video/x-msvideo"},
    {".mov", "video/quicktime"},
    {".woff", "font/woff"},
    {".woff2", "font/woff2"},
    {".ttf", "font/ttf"},
    {".otf", "font/otf"},
    {".eot", "application/vnd.ms-fontobject"}
  };

  const auto it = kMimeByExt.find(ext);
  if (it != kMimeByExt.end()) {
    return it->second;
  }

  return "application/octet-stream";
}


Error FilesystemEndpoint::handle(const network::ClientInfo& clientInfo, std::shared_ptr<HttpRequest> request)
{
  // 1. Read path.

  std::string_view requestPath = request->path();
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
    MINIROS_WARN("Client tried to access path \"%s\" out of prefix path", fsPath.u8string().c_str());
    return Error::FileNotFound;
  }

  std::string body;
  std::string type;
  if (std::filesystem::is_directory(fsPath)) {
    request->setResponseStatusOk();

    std::vector<std::filesystem::path> directories;
    std::vector<std::filesystem::path> files;

    for (const std::filesystem::path dirEntry : std::filesystem::directory_iterator(fsPath)) {
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
    type = "text/html";
  } else {
    // 2. Open and read file.
    std::ifstream in(fsPath);

    if (!in.is_open()) {
      return Error::FileNotFound;
    }

    std::istreambuf_iterator<char> it{in}, end;
    body.assign(it, end);
    type = contentTypeForFile(fsPath);
  }
  request->setResponseBody(body, type);
  // 3. Deliver.
  return Error::Ok;
}


}
}