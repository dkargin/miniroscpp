//
// Created by dkargin on 2/28/25.
//

#include <cassert>
#include <cstring>

#include "miniros/transport/http_tools.h"

#if defined(_MSC_VER)
# define strncasecmp	_strnicmp
#endif

namespace miniros {
namespace network {

template <int N>
bool tokenCmp(const char* a, const HttpFrame::Token& ta, const char b[N])
{
  if (ta.empty()) {
    return N == 0;
  }
  const int size = std::min<int>(ta.size(), N);
  return strncasecmp(a + ta.start,b, size) == 0;
}

HttpMethod HttpFrame::parseMethod(const char* data, const Token& token)
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

void HttpFrame::finish(bool request)
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

void HttpFrame::resetParseState(bool request)
{
  fields.clear();
  requestMethod = HttpMethod::Invalid;
  requestMethodToken = {};
  requestPath = {};
  requestHttpVersion = {};

  responseStatus = {};
  responseCodeToken = {};
  responseCode = 0;

  m_currentPosition = 0;
  m_bodyPosition = 0;
  m_bodyEnd = 0;
  m_tokenStart = 0;

  m_contentLength = -1;

  m_fieldName = {};

  m_keepAlive = true;

  m_state = request ? ParseRequestHeader : ParseResponseHeader;
}

std::string_view HttpFrame::getTokenView(const std::string& data, const Token& token)
{
  std::string_view result{};
  if (token.valid())
    result = std::string_view(data.c_str() + token.start, token.size());
  return result;
}

std::string_view HttpFrame::body() const
{
  int bodySize = bodyLength();
  if (bodySize > 0 && m_bodyPosition > 0)
    return std::string_view(data.c_str() + m_bodyPosition, bodySize);
  return {};
}

bool HttpFrame::hasContentLength() const
{
  return m_contentLength >= 0;
}

bool HttpFrame::keepAlive() const
{
  return m_keepAlive;
}

int HttpFrame::bodyLength() const
{
  if (m_bodyPosition <= 0 || m_contentLength < 0)
    return 0;

  int leftOver = data.length() - m_bodyPosition;
  if (leftOver < m_contentLength) {
    return leftOver;
  }

  return m_contentLength;
}

int HttpFrame::incrementalParse()
{
  const char *start = data.c_str();
  const char *end = start + data.length();
  const char* cp = start + m_currentPosition;
  const char* tokenStart = start + m_tokenStart;

  while (cp < end) {
    if (m_state == HttpFrame::ParseRequestHeader) {
      // Parsing request line: "GET /home.html HTTP/1.1"
      if (strncmp(cp, "\r\n", 2) == 0) {
        requestHttpVersion.assign(tokenStart - start, cp - start);
        cp += 2;
        tokenStart = cp;
        m_state = HttpFrame::ParseFieldName;
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
    else if (m_state == HttpFrame::ParseResponseHeader) {
      // Parsing response line: "200 OK"
      if (strncmp(cp, "\r\n", 2) == 0) {
        responseStatus.assign(tokenStart - start, cp - start);
        cp += 2;
        tokenStart = cp;
        m_state = HttpFrame::ParseFieldName;
        continue;
      } if (responseCodeToken.empty() && *cp == ' ') {
        responseCodeToken.assign(tokenStart - start, cp - start);
        responseCode = atoi(tokenStart);
        tokenStart = cp + 1;
      } else {
        // continue parsing.
      }
    }
    else if (m_state == HttpFrame::ParseFieldName) {
      if (*cp == ':') {
        m_state = HttpFrame::ParseFieldValue;
        m_fieldName.assign(tokenStart - start, cp - start);
        cp++;
        tokenStart = cp;
        continue;
      }

      if (strncmp(cp, "\n\n", 2) == 0 || strncmp(cp, "\r\n", 2) == 0) {
        m_state = HttpFrame::ParseBody;
        cp += 2;
        tokenStart = cp;
        m_bodyPosition = cp - start;
        break;
      }
    } else if (m_state == HttpFrame::ParseFieldValue) {
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
        } else {
          Field field;
          field.name = std::string(namePtr, m_fieldName.size());
          field.value = std::string(tokenStart, cp - tokenStart);
          fields.push_back(std::move(field));
        }
        cp += 2;
        tokenStart = cp;
        m_state = HttpFrame::ParseFieldName;
        continue;
      }
    } else // Any other state.
      break;
    cp++;
  }
  m_tokenStart = tokenStart - start;

  if (m_state == HttpFrame::ParseBody) {
    if (m_contentLength >= 0) {
      // Need body and got all the data.
      if (m_bodyPosition + m_contentLength <= data.length()) {
        m_state = HttpFrame::ParseComplete;
        m_bodyEnd = m_bodyPosition + m_contentLength;
        cp = start + m_bodyEnd;
      } else {
        // `data` does not contain full body yet, but need to advance `cp`.
        cp = start + data.length();
      }
    } else {
      m_state = HttpFrame::ParseComplete;
      m_bodyEnd = m_bodyPosition;
      cp = start + m_bodyEnd;
    }
  }

  int position = cp - start;
  int bytesParsed = position - m_currentPosition;
  m_currentPosition = position;

  return bytesParsed;
}

bool HttpFrame::hasHeader() const
{
  return m_state == HttpFrame::ParseBody || m_state == ParseComplete;
}

} // namespace network
} // namespace miniros
