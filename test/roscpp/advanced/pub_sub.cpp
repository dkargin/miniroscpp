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
 * Publish a message N times, back to back
 */

#include <string>
#include <cstdio>
#include <time.h>
#include <stdlib.h>

#include "miniros/ros.h"
#include <test_roscpp/TestArray.hxx>

int32_t g_array_size = 1;

#define USAGE "USAGE: publish_n_fast <sz>"

int main(int argc, char** argv)
{
  miniros::init(argc, argv, "pub_sub");

  if (argc < 2)
  {
    puts(USAGE);
    return EXIT_FAILURE;
  }

  g_array_size = atoi(argv[1]);

  miniros::NodeHandle nh;

  miniros::Publisher pub = nh.advertise<test_roscpp::TestArray>("roscpp/pubsub_test", 1);
  //miniros::Subscriber sub = nh.subscribe<test_roscpp::TestArray>("roscpp/subpub_test", 1, boost::bind(messageCallback, boost::placeholders::_1, pub));
  miniros::Subscriber sub = nh.subscribe<test_roscpp::TestArray>("roscpp/subpub_test", 1,
    [&pub](const test_roscpp::TestArrayConstPtr& msg){
    test_roscpp::TestArray copy = *msg;
      copy.counter++;

      while (miniros::ok() && pub.getNumSubscribers() == 0)
      {
        miniros::Duration(0.01).sleep();
      }

      pub.publish(copy);
    });

  miniros::spin();

  return EXIT_SUCCESS;
}
