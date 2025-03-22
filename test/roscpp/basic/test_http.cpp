//
// Created by dkargin on 3/22/25.
//

#include <gtest/gtest.h>

#include "miniros/transport/http_tools.h"
#include "miniros/transport/net_address.h"
#include "miniros/transport/url.h"

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
  network::HttpFrame httpFrame;

  std::string req(request3);
  httpFrame.data.append(req);

  int parsed = httpFrame.incrementalParse();

  EXPECT_EQ(parsed, req.size());
  EXPECT_EQ(httpFrame.state(), network::HttpFrame::ParseComplete);
}

TEST(net, parseFragmented)
{
  network::HttpFrame httpFrame;
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
  EXPECT_EQ(httpFrame.state(), network::HttpFrame::ParseComplete);
}


TEST(net, parseMultipleRequests)
{
  network::HttpFrame httpFrame;

  std::string req1(request1);
  std::string req2(request2);
  std::string req3(request3);

  httpFrame.data.append(req1);
  httpFrame.data.append(req2);
  httpFrame.data.append(req3);

  int parsed = httpFrame.incrementalParse();
  std::string_view body1 = httpFrame.body();
  EXPECT_EQ(parsed, req1.size());
  EXPECT_EQ(httpFrame.state(), network::HttpFrame::ParseComplete);
  httpFrame.finishReqeust();

  parsed = httpFrame.incrementalParse();
  std::string_view body2 = httpFrame.body();
  EXPECT_EQ(parsed, req2.size());
  EXPECT_EQ(httpFrame.state(), network::HttpFrame::ParseComplete);
  httpFrame.finishReqeust();

  parsed = httpFrame.incrementalParse();
  std::string_view body3 = httpFrame.body();
  EXPECT_EQ(parsed, req3.size());
  EXPECT_EQ(httpFrame.state(), network::HttpFrame::ParseComplete);
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
