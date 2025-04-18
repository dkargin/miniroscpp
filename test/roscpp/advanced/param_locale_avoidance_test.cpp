/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
********************************************************************/

#include "miniros/ros.h"
#include <locale.h>
#include <gtest/gtest.h>

TEST(Locale, push_pop)
{
  int argc = 0;
  char argv[1][255] = { "string" };
  miniros::init(argc, (char**)argv, "locale_push_pop");
  miniros::NodeHandle nh;

  MINIROS_INFO("locale is %s", setlocale(LC_NUMERIC, 0));

  // this test only works on machines that have de_DE installed
  if (!setlocale(LC_NUMERIC, "de_DE.utf8"))
    {
      MINIROS_WARN("unable to set locale to de_DE.utf8, skipping test");
      return;
    }
  MINIROS_INFO("locale now %s", setlocale(LC_NUMERIC, 0));
  for(unsigned j=0; miniros::ok() && j < 5; ++j)
  {
    MINIROS_INFO("setting parameters...");

    double set = 123.45;
    nh.setParam("V", set);

    double got;
    if (nh.getParam("V", got))
    {
      MINIROS_INFO("got pi=%3f <- should have commas in it", got);
      ASSERT_EQ(set, got);
    } else {
      FAIL();
    }

    miniros::WallDuration(0.1).sleep();
  }
}

int
main(int argc, char** argv)
{
  miniros::init(argc, argv, "locale_push_pop");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
