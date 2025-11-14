//
// Created by dkargin on 2/28/25.
//

#include <cassert>
#include <cstring>

#include "miniros/http/http_tools.h"

#if defined(_MSC_VER)
# define strncasecmp	_strnicmp
#endif

namespace miniros {
namespace http {

bool startsWith(const std::string_view& str, const std::string_view& prefix)
{
  if (str.size() < prefix.size())
    return false;
  if (prefix.empty())
    return true;
  return strncmp(str.data(), prefix.data(), prefix.size()) == 0;
}

template <unsigned int N>
bool tokenCmp(const char* a, const HttpParserFrame::Token& ta, const char b[N])
{
  if (ta.empty()) {
    return N == 0;
  }
  const size_t size = std::min<size_t>(ta.size(), N);
  return strncasecmp(a + ta.start,b, size) == 0;
}

void HttpParserFrame::Token::assign(int start_, int end_)
{
  assert(start_ <= end_);
  start = start_;
  end = end_;
}

HttpMethod HttpParserFrame::parseMethod(const char* data, const Token& token)
{
  if (tokenCmp<3>(data, token, "GET"))
    return HttpMethod::Get;
  if (tokenCmp<4>(data, token, "POST"))
    return HttpMethod::Post;
  if (tokenCmp<4>(data, token, "HEAD"))
    return HttpMethod::Head;
  if (tokenCmp<3>(data, token, "PUT"))
    return HttpMethod::Put;
  if (tokenCmp<6>(data, token, "DELETE"))
    return HttpMethod::Delete;
  if (tokenCmp<7>(data, token, "CONNECT"))
    return HttpMethod::Connect;
  if (tokenCmp<7>(data, token, "OPTIONS"))
    return HttpMethod::Options;
  if (tokenCmp<5>(data, token, "TRACE"))
    return HttpMethod::Trace;
  if (tokenCmp<5>(data, token, "PATCH"))
    return HttpMethod::Patch;
  return HttpMethod::Invalid;
}

const char* HttpMethod::toString() const
{
  switch (value) {
    case HttpMethod::Get: return "GET";
    case HttpMethod::Post: return "POST";
    case HttpMethod::Put: return "PUT";
    case HttpMethod::Delete: return "DELETE";
    case HttpMethod::Head: return "HEAD";
    case HttpMethod::Options: return "OPTIONS";
    case HttpMethod::Trace: return "TRACE";
    case HttpMethod::Patch: return "PATCH";
    case HttpMethod::Connect: return "CONNECT";
    case HttpMethod::Invalid: return "INVALID";
  }
  return "UNKNOWN";
}

void HttpParserFrame::finish(bool request)
{
  if (m_state == ParseComplete) {
    size_t currentLength = m_bodyPosition;
    if (m_contentLength >= 0)
      currentLength += m_contentLength;
    if (m_contentLength  == data.size()) {
      data.clear();
    } else {
      // Truncate all data related to current request
      std::string newData(data.data() + currentLength, data.size() - currentLength);
      std::swap(data, newData);
    }
  } else {
    data.clear();
  }
  resetParseState(request);
}

void HttpParserFrame::resetParseState(bool request)
{
  fields.clear();
  requestMethod = HttpMethod::Invalid;
  requestMethodToken = {};
  requestPath = {};
  protocol = {};

  responseStatus = {};
  responseCodeToken = {};
  responseCode = 0;

  contentType = {};
  m_currentPosition = 0;
  m_bodyPosition = 0;
  m_bodyEnd = 0;
  m_tokenStart = 0;

  m_contentLength = -1;

  m_fieldName = {};

  m_keepAlive = true;

  m_state = request ? ParseRequestHeader : ParseResponseHeader;
}

std::string_view HttpParserFrame::getTokenView(const std::string& data, const Token& token)
{
  std::string_view result{};
  if (token.valid())
    result = std::string_view(data.c_str() + token.start, token.size());
  return result;
}

std::string_view HttpParserFrame::header() const
{
  if (m_bodyPosition > 0)
    return std::string_view(data.c_str(), m_bodyPosition);
  return std::string_view(data.c_str(), data.size());
}

std::string_view HttpParserFrame::body() const
{
  int bodySize = bodyLength();
  if (bodySize > 0 && m_bodyPosition > 0)
    return std::string_view(data.c_str() + m_bodyPosition, bodySize);
  return {};
}

bool HttpParserFrame::hasContentLength() const
{
  return m_contentLength >= 0;
}

bool HttpParserFrame::keepAlive() const
{
  return m_keepAlive;
}

int HttpParserFrame::bodyLength() const
{
  if (m_bodyPosition <= 0 || m_contentLength < 0)
    return 0;

  int leftOver = static_cast<int>(data.length()) - m_bodyPosition;
  if (leftOver < m_contentLength) {
    return leftOver;
  }

  return m_contentLength;
}

int HttpParserFrame::incrementalParse()
{
  const char *start = data.c_str();
  const char *end = start + data.length();
  const char* cp = start + m_currentPosition;
  const char* tokenStart = start + m_tokenStart;

  while (cp < end && m_state != HttpParserFrame::ParseInvalid) {
    if (m_state == HttpParserFrame::ParseRequestHeader) {
      // Parsing request line: "GET /home.html HTTP/1.1"
      if (strncmp(cp, "\r\n", 2) == 0) {
        protocol.assign(tokenStart - start, cp - start);
        cp += 2;
        tokenStart = cp;
        m_state = HttpParserFrame::ParseFieldName;
        continue;
      } if (requestMethodToken.empty() && *cp == ' ') {
        requestMethodToken.assign(tokenStart - start, cp - start);
        requestMethod = parseMethod(start, requestMethodToken);
        tokenStart = cp + 1;
      } else if (requestPath.empty() && *cp == ' ') {
        requestPath.assign(tokenStart - start, cp - start);
        tokenStart = cp + 1;
      } else {
        // continue parsing.
      }
    }
    else if (m_state == HttpParserFrame::ParseResponseHeader) {
      // Parsing response line: "HTTP/1.1 200 OK\r\n" or "200 OK\r\n"
      if (strncmp(cp, "\r\n", 2) == 0) {
        if (!protocol.empty() && responseCodeToken.empty()) {
          std::swap(responseCodeToken, protocol);
          responseCode = atoi(data.c_str() + responseCodeToken.start);
        }
        // End of response line.
        if (responseCodeToken.empty()) {
          // We haven't found the status code yet - this shouldn't happen, but handle it
          // The entire tokenStart to cp is the status code
          responseCodeToken.assign(tokenStart - start, cp - start);
          responseCode = atoi(tokenStart);
        } else {
          // We have status code, so tokenStart to cp is the status text
          responseStatus.assign(tokenStart - start, cp - start);
        }

        if (responseCode != 0) {
          cp += 2;
          tokenStart = cp;
          m_state = HttpParserFrame::ParseFieldName;
        } else {
          m_state = HttpParserFrame::ParseInvalid;
        }
      } else if (*cp == ' ') {
        if (protocol.empty()) {
          // We hit a space - check if what we've parsed so far looks like HTTP version
          int tokenLen = cp - tokenStart;
          protocol.assign(tokenStart - start, cp - start);
          tokenStart = cp + 1;  // Skip the space, start of status code.
        } else if (responseCodeToken.empty()) {
          // Found space after status code (after HTTP version was already parsed)
          responseCodeToken.assign(tokenStart - start, cp - start);
          responseCode = atoi(tokenStart);
          tokenStart = cp + 1;  // Start of status text
        } else {
          // Continue parsing (either HTTP version, status code, or status text)
        }
      }
    }
    else if (m_state == HttpParserFrame::ParseFieldName) {
      if (*cp == ':') {
        m_state = HttpParserFrame::ParseFieldValue;
        m_fieldName.assign(tokenStart - start, cp - start);
        cp++;
        while (cp < end && *cp == ' ') {
          cp++;
        }
        tokenStart = cp;
        continue;
      }

      if (strncmp(cp, "\n\n", 2) == 0 || strncmp(cp, "\r\n", 2) == 0) {
        m_state = HttpParserFrame::ParseBody;
        cp += 2;
        tokenStart = cp;
        m_bodyPosition = cp - start;
        break;
      }
    } else if (m_state == HttpParserFrame::ParseFieldValue) {
      assert(tokenStart != nullptr);
      if (strncmp(cp, "\r\n", 2) == 0) {
        const char* namePtr = start + m_fieldName.start;
        if (strncasecmp(namePtr, "Content-length", 14) == 0)  {
          m_contentLength = atoi(tokenStart);
        } else if (strncasecmp(namePtr, "Connection", 10) == 0) {
          if (strncasecmp(tokenStart, "keep-alive", 10) == 0) {
            m_keepAlive = true;
          } else if (strncasecmp(tokenStart, "close", 5) == 0) {
            m_keepAlive = false;
          }
        } else if (strncasecmp(namePtr, "Content-Type", 12) == 0) {
          contentType.assign(tokenStart - start, cp - start);
        } else {
          /// Unspecialized field
          Field field;
          field.name = std::string(namePtr, m_fieldName.size());
          field.value = std::string(tokenStart, cp - tokenStart);
          fields.push_back(std::move(field));
        }
        cp += 2;
        tokenStart = cp;
        m_state = HttpParserFrame::ParseFieldName;
        continue;
      }
    } else // Any other state.
      break;
    cp++;
  }
  m_tokenStart = tokenStart - start;

  if (m_state == HttpParserFrame::ParseBody) {
    if (m_contentLength >= 0) {
      // Need body and got all the data.
      if (m_bodyPosition + m_contentLength <= data.length()) {
        m_state = HttpParserFrame::ParseComplete;
        m_bodyEnd = m_bodyPosition + m_contentLength;
        cp = start + m_bodyEnd;
      } else {
        // `data` does not contain full body yet, but need to advance `cp`.
        cp = start + data.length();
      }
    } else {
      m_state = HttpParserFrame::ParseComplete;
      m_bodyEnd = m_bodyPosition;
      cp = start + m_bodyEnd;
    }
  }

  int position = cp - start;
  int bytesParsed = position - m_currentPosition;
  m_currentPosition = position;

  return bytesParsed;
}

bool HttpParserFrame::hasHeader() const
{
  return m_state == HttpParserFrame::ParseBody || m_state == ParseComplete;
}

void HttpResponseHeader::reset()
{
  statusCode = 200;
  contentType.clear();
  server.clear();
  status.clear();
}

void HttpResponseHeader::writeHeader(std::string& output, size_t bodySize) const
{
  if (!protocol.empty()) {
    output += protocol;
    output += ' ';
  }

  output += std::to_string(statusCode);
  output += " ";
  output += status;
  output += "\r\n";

  if (!server.empty()) {
    output += "Server: ";
    output += server; // XMLRPC_VERSION
    output += "\r\n";
  }
  output += "Content-Type: ";
  output += contentType;
  output += "\r\n";

  if (bodySize > 0) {
    output += "Content-Length: ";
    output += std::to_string(bodySize);
    output += "\r\n";
  }

  output += "\r\n";
}

} // namespace network
} // namespace miniros
