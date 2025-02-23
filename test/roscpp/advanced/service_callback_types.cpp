
/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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

/**
 * Author: Josh Faust
 */

/*
 * Test compilation of all the different service callback types
 */

#include <gtest/gtest.h>
#include "miniros/ros.h"
#include "test_roscpp/TestStringString.hxx"

#include <vector>

bool add(test_roscpp::TestStringString::Request &,
         test_roscpp::TestStringString::Response &)
{
  return true;
}

bool add2(miniros::ServiceEvent<test_roscpp::TestStringString::Request, test_roscpp::TestStringString::Response>&)
{
  return true;
}

bool add3(miniros::ServiceEvent<test_roscpp::TestStringString::Request, test_roscpp::TestStringString::Response>&, const std::string&)
{
  return true;
}

struct A
{
  bool add(test_roscpp::TestStringString::Request &,
           test_roscpp::TestStringString::Response &)
  {
    return true;
  }

  bool add2(miniros::ServiceEvent<test_roscpp::TestStringString::Request, test_roscpp::TestStringString::Response>&)
  {
    return true;
  }

  bool add3(miniros::ServiceEvent<test_roscpp::TestStringString::Request, test_roscpp::TestStringString::Response>&, const std::string&)
  {
    return true;
  }
};

TEST(ServiceCallbackTypes, compile)
{
  miniros::NodeHandle n;

  std::vector<miniros::ServiceServer> srvs;
  srvs.push_back(n.advertiseService("add_two_ints", add));
  srvs.push_back(n.advertiseService("add_two_ints2", add2));
  //srvs.push_back(n.advertiseService<miniros::ServiceEvent<test_roscpp::TestStringString::Request, test_roscpp::TestStringString::Response> >("add_two_ints3", boost::bind(add3, boost::placeholders::_1, std::string("blah"))));
  srvs.push_back(n.advertiseService<miniros::ServiceEvent<test_roscpp::TestStringString::Request, test_roscpp::TestStringString::Response> >("add_two_ints3",
    [](miniros::ServiceEvent<test_roscpp::TestStringString::Request, test_roscpp::TestStringString::Response>& r) {
      return add3(r, std::string("blah"));
    }));


  // TODO: Probably this signature is no different from previous test. And since boost::bind is not used, both of them
  // use trivial lambda for a call.
  A a;
  srvs.push_back(n.advertiseService("add_two_ints10", &A::add, &a));
  srvs.push_back(n.advertiseService("add_two_ints11", &A::add2, &a));
  //srvs.push_back(n.advertiseService<miniros::ServiceEvent<test_roscpp::TestStringString::Request, test_roscpp::TestStringString::Response> >("add_two_ints12", boost::bind(&A::add3, &a, boost::placeholders::_1, std::string("blah"))));
  srvs.push_back(n.advertiseService<miniros::ServiceEvent<test_roscpp::TestStringString::Request, test_roscpp::TestStringString::Response> >("add_two_ints12",
    [&a](miniros::ServiceEvent<test_roscpp::TestStringString::Request, test_roscpp::TestStringString::Response>& r) {
      return a.add3(r, std::string("blah"));
    }));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  miniros::init( argc, argv, "subscription_callback_types" );
  miniros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
