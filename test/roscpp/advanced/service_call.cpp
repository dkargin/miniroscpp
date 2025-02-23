/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Brian Gerkey */

/*
 * Call a service
 */

#include <string>

#include <gtest/gtest.h>

#include "miniros/ros.h"
#include "miniros/service.h"
#include "miniros/transport/connection.h"
#include "miniros/service_client.h"
#include <test_roscpp/TestStringString.hxx>
#include <test_roscpp/BadTestStringString.hxx>

TEST(SrvCall, callSrv)
{
  test_roscpp::TestStringString::Request req;
  test_roscpp::TestStringString::Response res;

  req.str = std::string("case_FLIP");

  ASSERT_TRUE(miniros::service::waitForService("service_adv"));
  ASSERT_TRUE(miniros::service::call("service_adv", req, res));

  ASSERT_STREQ(res.str.c_str(), "CASE_flip");
}

TEST(SrvCall, callSrvUnicode)
{
  test_roscpp::TestStringString::Request req;
  test_roscpp::TestStringString::Response res;

  req.str = std::string("ロボット");

  ASSERT_TRUE(miniros::service::waitForService("service_adv"));
  ASSERT_TRUE(miniros::service::call("service_adv", req, res));

  ASSERT_STREQ(res.str.c_str(), "ロボット");
}

TEST(SrvCall, callSrvMultipleTimes)
{
  test_roscpp::TestStringString::Request req;
  test_roscpp::TestStringString::Response res;

  req.str = std::string("case_FLIP");

  ASSERT_TRUE(miniros::service::waitForService("service_adv"));

  miniros::Time start = miniros::Time::now();

  for (int i = 0; i < 100; ++i)
  {
    ASSERT_TRUE(miniros::service::call("service_adv", req, res));
  }

  miniros::Time end = miniros::Time::now();
  miniros::Duration d = end - start;
  printf("100 calls took %f secs\n", d.toSec());

  ASSERT_STREQ(res.str.c_str(), "CASE_flip");
}

TEST(SrvCall, callSrvWithWrongType)
{
  test_roscpp::BadTestStringString::Request req;
  test_roscpp::BadTestStringString::Response res;

  ASSERT_TRUE(miniros::service::waitForService("service_adv"));

  for ( int i = 0; i < 4; ++i )
  {
    bool call_result = miniros::service::call("service_adv", req, res);
    ASSERT_FALSE(call_result);
  }
}

TEST(SrvCall, callSrvHandle)
{
  test_roscpp::TestStringString::Request req;
  test_roscpp::TestStringString::Response res;

  req.str = std::string("case_FLIP");

  std::map<std::string, std::string> header;
  header["test1"] = "testing 1";
  header["test2"] = "testing 2";
  miniros::NodeHandle nh;
  miniros::ServiceClient handle = nh.serviceClient<test_roscpp::TestStringString>("service_adv", false, header);
  ASSERT_TRUE(handle.waitForExistence());

  miniros::Time start = miniros::Time::now();

  for (int i = 0; i < 100; ++i)
  {
    ASSERT_TRUE(handle.call(req, res));
  }

  miniros::Time end = miniros::Time::now();
  miniros::Duration d = end - start;
  printf("100 calls took %f secs\n", d.toSec());

  ASSERT_STREQ(res.str.c_str(), "CASE_flip");
}

TEST(SrvCall, callSrvPersistentHandle)
{
  test_roscpp::TestStringString::Request req;
  test_roscpp::TestStringString::Response res;

  req.str = std::string("case_FLIP");

  ASSERT_TRUE(miniros::service::waitForService("service_adv"));

  std::map<std::string, std::string> header;
  header["test1"] = "testing 1";
  header["test2"] = "testing 2";
  miniros::NodeHandle nh;
  miniros::ServiceClient handle = nh.serviceClient<test_roscpp::TestStringString>("service_adv", true, header);

  miniros::Time start = miniros::Time::now();

  for (int i = 0; i < 10000; ++i)
  {
    ASSERT_TRUE(handle.call(req, res));
  }

  miniros::Time end = miniros::Time::now();
  miniros::Duration d = end - start;
  printf("10000 calls took %f secs\n", d.toSec());

  ASSERT_STREQ(res.str.c_str(), "CASE_flip");
}

TEST(SrvCall, callSrvLongRunning)
{
  test_roscpp::TestStringString::Request req;
  test_roscpp::TestStringString::Response res;

  req.str = std::string("case_FLIP");

  ASSERT_TRUE(miniros::service::waitForService("service_adv_long"));
  ASSERT_TRUE(miniros::service::call("service_adv_long", req, res));

  ASSERT_STREQ(res.str.c_str(), "CASE_flip");
}

TEST(SrvCall, callSrvWhichUnadvertisesInCallback)
{
  test_roscpp::TestStringString::Request req;
  test_roscpp::TestStringString::Response res;

  req.str = std::string("case_FLIP");

  ASSERT_TRUE(miniros::service::waitForService("service_adv_unadv_in_callback"));
  ASSERT_FALSE(miniros::service::call("service_adv_unadv_in_callback", req, res));
}

TEST(SrvCall, handleValid)
{
  test_roscpp::TestStringString::Request req;
  test_roscpp::TestStringString::Response res;

  req.str = std::string("case_FLIP");

  ASSERT_TRUE(miniros::service::waitForService("service_adv"));

  std::map<std::string, std::string> header;
  header["test1"] = "testing 1";
  header["test2"] = "testing 2";
  miniros::NodeHandle nh;
  miniros::ServiceClient handle = nh.serviceClient<test_roscpp::TestStringString>("service_adv", true, header);
  ASSERT_TRUE(handle.call(req, res));
  ASSERT_TRUE(handle.isValid());
  handle.shutdown();
  ASSERT_FALSE(handle.isValid());

  ASSERT_STREQ(res.str.c_str(), "CASE_flip");
}

TEST(SrvCall, waitForServiceTimeout)
{
  miniros::NodeHandle nh;
  ASSERT_FALSE(miniros::service::waitForService("iojergoiwjoiewg", 1000));
  ASSERT_FALSE(miniros::service::waitForService("iojergoiwjoiewg", miniros::Duration(1)));

  miniros::ServiceClient handle = nh.serviceClient<test_roscpp::TestStringString>("migowiowejowieuhwejg", false);
  ASSERT_FALSE(handle.waitForExistence(miniros::Duration(1)));
}

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  miniros::init(argc, argv, "service_call");
  miniros::NodeHandle nh;

  int ret = RUN_ALL_TESTS();



  return ret;
}


