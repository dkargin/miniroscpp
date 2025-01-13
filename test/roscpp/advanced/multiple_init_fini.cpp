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

/* Author: Josh Faust */

/*
 * Repeatedly create and destroy a node.  Do some stuff in between to be sure things are working
 */

#include <string>

#include <gtest/gtest.h>

#include <time.h>
#include <stdlib.h>

#include "miniros/ros.h"
#include "miniros/transport/callback_queue.h"

#include <test_roscpp/TestArray.hxx>

int g_argc;
char** g_argv;

bool g_got_callback = false;

void callback(const test_roscpp::TestArrayConstPtr&)
{
  g_got_callback = true;
}

TEST(roscpp, multipleInitAndFini)
{
  int try_count = 10;
  if ( g_argc > 1 )
  {
    try_count = atoi(g_argv[1]);
  }

  for ( int i = 0; i < try_count; ++i )
  {
    g_got_callback = false;

    miniros::init( g_argc, g_argv, "multiple_init_fini" );
    miniros::NodeHandle nh;

    miniros::Subscriber sub = nh.subscribe("test", 1, callback);
    ASSERT_TRUE(sub);

    miniros::Publisher pub = nh.advertise<test_roscpp::TestArray>( "test", 1 );
    ASSERT_TRUE(pub);

    pub.publish(test_roscpp::TestArray());
    miniros::getGlobalCallbackQueue()->callAvailable(miniros::WallDuration(1));

    ASSERT_TRUE(g_got_callback) << "did not receive a message in iteration " << i;

    miniros::shutdown();
  }
}

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  g_argc = argc;
  g_argv = argv;
  return RUN_ALL_TESTS();
}
