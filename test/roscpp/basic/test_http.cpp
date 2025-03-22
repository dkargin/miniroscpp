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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
