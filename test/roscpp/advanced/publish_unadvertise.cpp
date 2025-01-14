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

#include <string>

#include <gtest/gtest.h>

#include <time.h>
#include <stdlib.h>

#include "miniros/ros.h"
#include <test_roscpp/TestArray.hxx>

static int g_argc;
static char** g_argv;

bool failure;
bool advertised;

class Publications : public testing::Test
{
public:
  miniros::NodeHandle nh_;
  miniros::Publisher pub_;

  void subscriberCallback(const miniros::SingleSubscriberPublisher&)
  {
    MINIROS_INFO("subscriberCallback invoked");
    if(!advertised)
    {
      MINIROS_INFO("but not advertised");
      failure = true;
    }
  }

  bool adv()
  {
    pub_ = nh_.advertise<test_roscpp::TestArray>("roscpp/pubsub_test", 1, boost::bind(&Publications::subscriberCallback, this, boost::placeholders::_1));
    return pub_;
  }

  void unadv()
  {
    pub_.shutdown();
  }

protected:
  Publications() {}
  void SetUp()
  {
    advertised = false;
    failure = false;

    ASSERT_TRUE(g_argc == 1);
  }
  void TearDown()
  {
  }
};

TEST_F(Publications, pubUnadvertise)
{
  advertised = true;
  MINIROS_INFO("advertising");
  ASSERT_TRUE(adv());
  miniros::Time t1(miniros::Time::now()+miniros::Duration(2.0));

  while(miniros::Time::now() < t1 && !failure)
  {
    miniros::WallDuration(0.01).sleep();
    miniros::spinOnce();
  }

  unadv();

  MINIROS_INFO("unadvertised");
  advertised = false;

  miniros::Time t2(miniros::Time::now()+miniros::Duration(2.0));
  while(miniros::Time::now() < t2 && !failure)
  {
    miniros::WallDuration(0.01).sleep();
    miniros::spinOnce();
  }

  advertised = true;
  ASSERT_TRUE(adv());
  unadv();
  ASSERT_TRUE(adv());

  if(failure)
    FAIL();
  else
    SUCCEED();
}

int
main(int argc, char** argv)
{
  miniros::init(argc, argv, "publish_unadvertise");
  testing::InitGoogleTest(&argc, argv);
  g_argc = argc;
  g_argv = argv;
  return RUN_ALL_TESTS();
}
