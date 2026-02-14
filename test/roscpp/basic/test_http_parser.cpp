//
// Created by dkargin on 3/22/25.
//

#include <gtest/gtest.h>

#include "miniros/http/http_tools.h"
#include "miniros/http/http_endpoint.h"
#include "miniros/http/http_filters.h"

#include "miniros/network/url.h"

#include "gtest_printers.h"

using namespace miniros;

TEST(net, parseGoodURL)
{
  const std::string strUrlWithPort = "http://localhost:8080/RPC2";

  network::URL url;
  ASSERT_TRUE(url.fromString(strUrlWithPort, false));

  EXPECT_STREQ(strUrlWithPort.c_str(), url.str().c_str());
  EXPECT_EQ(url.port, 8080);
  EXPECT_STREQ(url.path.c_str(), "/RPC2");

  const std::string urlWithQuery = "http://182.158.141.2:80/foo.html?&q=1:2:3";
  ASSERT_TRUE(url.fromString(urlWithQuery, false));
  EXPECT_EQ(url.port, 80);
  EXPECT_STREQ(url.host.c_str(), "182.158.141.2");
  EXPECT_STREQ(urlWithQuery.c_str(), url.str().c_str());

  const std::string urlWithQuery2 = "http://182.158.141.2:80?&q=1:2:3";
  ASSERT_TRUE(url.fromString(urlWithQuery2, false));
  EXPECT_EQ(url.port, 80);
  EXPECT_STREQ(url.host.c_str(), "182.158.141.2");
  EXPECT_STREQ(urlWithQuery2.c_str(), url.str().c_str());

  const std::string simplestURL = "hostname";
  ASSERT_TRUE(url.fromString(simplestURL, false));
  EXPECT_EQ(url.port, 0);
  EXPECT_TRUE(url.scheme.empty());
}

TEST(net, parseProblematicURL)
{
  const std::string urlOnlyPath = "/RPC2";
  network::URL url;
  ASSERT_TRUE(url.fromString(urlOnlyPath, false));
  EXPECT_STREQ(url.path.c_str(), "/RPC2");
}

const char* request1 = "POST /RPC2 HTTP/1.1\r\n" \
  "Host: localhost:11311\r\n" \
  "Accept-Encoding: gzip\r\n" \
  "Content-Type: text/xml\r\n" \
  "User-Agent: Python-xmlrpc/3.12\r\n" \
  "Content-Length: 167\r\n" \
  "\r\n" \
  "<?xml version='1.0'?>\n" \
  "<methodCall>\n" \
  "<methodName>getSystemState</methodName>\n" \
  "<params>\n" \
  "<param>\n" \
  "<value><string>/rostopic</string></value>\n" \
  "</param>\n" \
  "</params>\n" \
  "</methodCall>\n";

const char* request2 =
  "POST /RPC2 HTTP/1.1\r\n"
  "Host: localhost:11311\r\n"
  "Accept-Encoding: gzip\r\n"
  "Content-Type: text/xml\r\n"
  "User-Agent: Python-xmlrpc/3.12\r\n"
  "Content-Length: 166\r\n"
  "\r\n"
  "<?xml version='1.0'?>\n"
  "<methodCall>\n"
  "<methodName>getTopicTypes</methodName>\n"
  "<params>\n"
  "<param>\n"
  "<value><string>/rostopic</string></value>\n"
  "</param>\n"
  "</params>\n"
  "</methodCall>\n";

// Note slightly different line endings in request body. It has \r\n line endings, contrary to bodies
// of request1 and request2.
const char* request3 = "POST / HTTP/1.1\r\n"
  "User-Agent: XMLRPC++ 0.7\r\n"
  "Host: localhost:11311\r\n"
  "Content-Type: text/xml\r\n"
  "Content-length: 183\r\n"
  "\r\n"
  "<?xml version=\"1.0\"?>\r\n"
  "<methodCall><methodName>hasParam</methodName>\r\n"
  "<params><param><value>/rosout</value></param><param><value>/tcp_keepalive</value></param></params></methodCall>\r\n";

TEST(net, parseRequest)
{
  http::HttpParserFrame httpFrame;

  httpFrame.resetParseState(true);
  std::string req(request3);
  httpFrame.data.append(req);

  int parsed = httpFrame.incrementalParse();

  EXPECT_EQ(parsed, req.size());
  EXPECT_EQ(httpFrame.state(), http::HttpParserFrame::ParseComplete);
  EXPECT_EQ(httpFrame.requestMethod, http::HttpMethod::Post);
}

TEST(net, parseFragmented)
{
  http::HttpParserFrame httpFrame;
  httpFrame.resetParseState(true);

  std::string fullReq(request3);

  const int N = 3;
  size_t chunkSize = fullReq.size() / N;

  size_t totalParsed = 0;
  for (size_t i = 0; i < fullReq.size(); i += chunkSize) {
    size_t next = i + chunkSize;
    if (next >= fullReq.size()) {
      next = fullReq.size();
    }
    std::string part = fullReq.substr(i, next - i);
    httpFrame.data.append(part);

    int parsed = httpFrame.incrementalParse();
    EXPECT_GT(parsed, 0);
    totalParsed += parsed;
  }

  EXPECT_EQ(totalParsed, fullReq.size());
  EXPECT_EQ(httpFrame.state(), http::HttpParserFrame::ParseComplete);
  EXPECT_EQ(httpFrame.requestMethod, http::HttpMethod::Post);
}


TEST(net, parseMultipleRequests)
{
  http::HttpParserFrame httpFrame;
  httpFrame.resetParseState(true);

  std::string req1(request1);
  std::string req2(request2);
  std::string req3(request3);

  httpFrame.data.append(req1);
  httpFrame.data.append(req2);
  httpFrame.data.append(req3);

  int parsed = httpFrame.incrementalParse();
  std::string_view body1 = httpFrame.body();
  EXPECT_EQ(parsed, req1.size());
  EXPECT_EQ(httpFrame.state(), http::HttpParserFrame::ParseComplete);
  EXPECT_EQ(httpFrame.contentLength(), 167);
  EXPECT_EQ(httpFrame.requestMethod, http::HttpMethod::Post);
  httpFrame.finishRequest();

  parsed = httpFrame.incrementalParse();
  std::string_view body2 = httpFrame.body();
  EXPECT_EQ(parsed, req2.size());
  EXPECT_EQ(httpFrame.state(), http::HttpParserFrame::ParseComplete);
  httpFrame.finishRequest();

  parsed = httpFrame.incrementalParse();
  std::string_view body3 = httpFrame.body();
  EXPECT_EQ(parsed, req3.size());
  EXPECT_EQ(httpFrame.state(), http::HttpParserFrame::ParseComplete);
}


const char* response1 =
  "200 OK\r\n"
  "Access-Control-Allow-Origin: *\r\n"
  "Connection: Keep-Alive\r\n"
  "Content-Encoding: gzip\r\n"
  "Content-Type: text/html; charset=utf-8\r\n"
  "Date: Mon, 18 Jul 2016 16:06:00 GMT"
  "Etag: \"c561c68d0ba92bbeb8b0f612a9199f722e3a621a\"\r\n"
  "Keep-Alive: timeout=5, max=997\r\n"
  "Last-Modified: Mon, 18 Jul 2016 02:36:04 GMT\r\n"
  "Server: Apache\r\n"
  "Set-Cookie: my-key=my value; expires=Mon, 17-Jul-2017 16:06:00 GMT; Max-Age=31449600; Path=/; secure\r\n"
  "Transfer-Encoding: chunked\r\n"
  "Vary: Cookie, Accept-Encoding\r\n"
  "X-Backend-Server: developer2.webapp.scl3.mozilla.com\r\n"
  "X-Cache-Info: not cacheable; meta data too large\r\n"
  "X-kuma-revision: 1085259\r\n"
  "x-frame-options: DENY\r\n"
  "\r\n";

const char* response2 =
"RTSP/1.0 200 OK\r\n"
"Access-Control-Allow-Origin: *\r\n"
"Connection: Keep-Alive\r\n"
"Content-Encoding: gzip\r\n"
"\r\n";
TEST(net, parseResponse)
{
  http::HttpParserFrame httpFrame;

  httpFrame.resetParseState(false);
  const std::string rep1(response1);
  httpFrame.data.append(response1);

  size_t parsed = httpFrame.incrementalParse();

  EXPECT_EQ(parsed, rep1.size());
  EXPECT_EQ(httpFrame.state(), http::HttpParserFrame::ParseComplete);
  EXPECT_EQ(httpFrame.responseCode, 200);

  httpFrame.finish(false);
  const std::string rep2(response2);
  httpFrame.data.append(response2);
  parsed = httpFrame.incrementalParse();
  EXPECT_EQ(parsed, rep2.size());
  EXPECT_EQ(httpFrame.state(), http::HttpParserFrame::ParseComplete);
  EXPECT_EQ(httpFrame.responseCode, 200);
}

const char* badResponse =
"Completely Not Http response with strange number OK\r\n"
"\r\n";
TEST(net, parseBadResponse)
{
  http::HttpParserFrame httpFrame;

  httpFrame.resetParseState(false);
  const std::string req(badResponse);
  httpFrame.data.append(badResponse);

  int parsed = httpFrame.incrementalParse();

  EXPECT_GE(parsed, 0);
  EXPECT_EQ(httpFrame.state(), http::HttpParserFrame::ParseInvalid);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  miniros::handleCrashes();

  return RUN_ALL_TESTS();
}
