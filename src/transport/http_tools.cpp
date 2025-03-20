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

void HttpFrame::finishReqeust()
{
  if (m_state == ParseBody) {
    assert(data.length() == m_bodyPosition + m_contentLength);
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
  resetParseState();
}

void HttpFrame::resetParseState()
{
  fields.clear();
  requestType = {};
  requestUrl = {};
  requestHttpVersion = {};

  m_currentPosition = 0;
  m_bodyPosition = 0;
  m_tokenStart = 0;

  m_contentLength = -1;

  m_fieldName = {};

  m_keepAlive = true;

  m_state = HttpFrame::ParseRequest;
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

  if (m_state != HttpFrame::ParseRequest) {
    std::cout << "Double entrance" << std::endl;
  }

  for (; cp < end;) {
    if (m_state == HttpFrame::ParseRequest) {
      if (strncmp(cp, "\r\n", 2) == 0) {
        requestHttpVersion.assign(tokenStart - start, cp - start);
        cp += 2;
        tokenStart = cp;
        m_state = HttpFrame::ParseFieldName;
        continue;
      } if (requestType.empty() && *cp == ' ') {
        requestType.assign(tokenStart - start, cp - start);
        tokenStart = cp + 1;
      } else if (requestUrl.empty() && *cp == ' ') {
        requestUrl.assign(tokenStart - start, cp - start);
        tokenStart = cp + 1;
      } else {
        // Error?
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
    } else
      break;
    cp++;
  }

  int position = cp - start;
  int bytesParsed = position - m_currentPosition;

  m_tokenStart = tokenStart - start;
  m_currentPosition = position;

  return bytesParsed;
}

} // namespace network
} // namespace miniros
