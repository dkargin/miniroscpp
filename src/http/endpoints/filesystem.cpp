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

void FilesystemEndpoint::enableImageView(bool flag)
{
  image_view_enabled_ = flag;
}

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
    const std::string mimeType = contentTypeForFile(fsPath);
    const bool isImage = mimeType.rfind("image/", 0) == 0;
    const bool requestRawImage = request->getParameter("raw") == "1";

    if (image_view_enabled_ && isImage && !requestRawImage) {
      std::filesystem::path relativePath = std::filesystem::path(path);
      std::filesystem::path parentPath = relativePath.parent_path();
      std::filesystem::path currentName = relativePath.filename();

      std::vector<std::filesystem::path> imageFiles;
      for (const std::filesystem::path& dirEntry : std::filesystem::directory_iterator(fsPath.parent_path())) {
        if (!std::filesystem::is_regular_file(dirEntry)) {
          continue;
        }
        std::filesystem::path entry = dirEntry.filename();
        if (!show_hidden_ && !entry.empty()) {
          const std::string str = entry.string();
          if (!str.empty() && str[0] == '.') {
            continue;
          }
        }

        if (contentTypeForFile(dirEntry).rfind("image/", 0) == 0) {
          imageFiles.push_back(entry);
        }
      }
      std::sort(imageFiles.begin(), imageFiles.end());

      const auto currentIt = std::find(imageFiles.begin(), imageFiles.end(), currentName);

      auto makeFileUrl = [this, &parentPath](const std::filesystem::path& fileName) {
        const std::filesystem::path fullPath = parentPath / fileName;
        const std::string relative = fullPath.generic_string();
        if (prefix_path_.empty() || prefix_path_ == "/") {
          return std::string("/") + relative;
        }
        if (prefix_path_.back() == '/') {
          return prefix_path_ + relative;
        }
        return prefix_path_ + "/" + relative;
      };

      const std::string currentUrl = makeFileUrl(currentName);
      const std::string imageDataUrl = currentUrl + "?raw=1";

      std::string firstUrl;
      std::string prevUrl;
      std::string nextUrl;
      std::string lastUrl;
      if (!imageFiles.empty()) {
        firstUrl = makeFileUrl(imageFiles.front());
        lastUrl = makeFileUrl(imageFiles.back());
      }
      if (currentIt != imageFiles.end() && currentIt != imageFiles.begin()) {
        prevUrl = makeFileUrl(*(currentIt - 1));
      }
      if (currentIt != imageFiles.end() && (currentIt + 1) != imageFiles.end()) {
        nextUrl = makeFileUrl(*(currentIt + 1));
      }

      auto navButton = [](const char* label, const std::string& url) {
        if (url.empty()) {
          return std::string("<button disabled>") + label + "</button>";
        }
        return std::string("<a href=\"") + url + "\"><button>" + label + "</button></a>";
      };

      std::stringstream ss;
      ss << "<!doctype html><html><head><meta charset=\"utf-8\"/>"
         << "<title>" << currentName.string() << "</title></head><body>";
      ss << "<h2>" << currentName.string() << "</h2>";
      ss << "<p>"
         << navButton("first", firstUrl) << " "
         << navButton("prev", prevUrl) << " "
         << navButton("next", nextUrl) << " "
         << navButton("last", lastUrl)
         << "</p>";
      ss << "<div><img src=\"" << imageDataUrl
         << "\" alt=\"" << currentName.string()
         << "\" style=\"max-width:100%;max-height:90vh;\"/></div>";
      ss << "</body></html>";

      body = ss.str();
      type = "text/html";
    } else {
    std::ifstream in(fsPath);

    if (!in.is_open()) {
      return Error::FileNotFound;
    }

    std::istreambuf_iterator<char> it{in}, end;
    body.assign(it, end);
    type = mimeType;
    }
  }
  request->setResponseBody(body, type);
  // 3. Deliver.
  return Error::Ok;
}


}
}