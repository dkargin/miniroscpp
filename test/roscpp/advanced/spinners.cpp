/*
 * Copyright (c) 2011, Willow Garage, Inc.
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

/* Author: Troy Straszheim */

/*
 * Spinny spinner basic tests.
 
 these do NOT ACTUALLY TEST ANYTHING as it is diffcult to restart all
 of ros in a unit test, and there is no way to test (that I can see
 right now) that certain error messages are emitted, nor any way to
 specify that a test should fail or return an error.  So this is just
 a placeholder to be run manually, one test at a time (via
 --gtest_filter) when the next problem occurs.  Those that end with
 'fail' actually success, but they send a fatal error message that
 you're trying to spin from multiple threads.
*/

#include <gtest/gtest.h>
#include <thread>

#include "miniros/spinner.h"
#include "miniros/init.h"
#include "miniros/node_handle.h"

using namespace miniros;

int argc_;
char** argv_;

void fire_shutdown(const miniros::WallTimerEvent&) {
  MINIROS_INFO("Asking for shutdown");
  miniros::shutdown();
}

#define DOIT()                                                  \
  miniros::init(argc_, argv_, "test_spinners");                     \
  NodeHandle nh;                                                \
  miniros::WallTimer t = nh.createWallTimer(miniros::WallDuration(2.0), \
                                        &fire_shutdown);        \
  
TEST(Spinners, spin)
{
  DOIT();
  miniros::spin(); // will block until ROS shutdown
}

TEST(Spinners, spinfail)
{
  DOIT();
  std::thread th([]() {miniros::spin(); });
  miniros::WallDuration(0.1).sleep(); // wait for thread to be started

  EXPECT_THROW(miniros::spin(), std::runtime_error);

  SingleThreadedSpinner ss;
  EXPECT_THROW(miniros::spin(ss), std::runtime_error);
  EXPECT_THROW(ss.spin(), std::runtime_error);

  MultiThreadedSpinner ms;
  EXPECT_THROW(miniros::spin(ms), std::runtime_error);
  EXPECT_THROW(ms.spin(), std::runtime_error);

  AsyncSpinner as(2);
  EXPECT_THROW(as.start(), std::runtime_error);

  miniros::waitForShutdown();
  th.join();
}

TEST(Spinners, singlefail)
{
  DOIT();
  SingleThreadedSpinner ss;
  std::thread th([&ss]() {miniros::spin(ss); });
  miniros::WallDuration(0.1).sleep(); // wait for thread to be started

  EXPECT_THROW(miniros::spin(), std::runtime_error);

  SingleThreadedSpinner ss2;
  EXPECT_THROW(miniros::spin(ss2), std::runtime_error);
  EXPECT_THROW(ss2.spin(), std::runtime_error);

  MultiThreadedSpinner ms;
  EXPECT_THROW(miniros::spin(ms), std::runtime_error);
  EXPECT_THROW(ms.spin(), std::runtime_error);

  AsyncSpinner as(2);
  EXPECT_THROW(as.start(), std::runtime_error);

  miniros::waitForShutdown();
  th.join();
}

TEST(Spinners, multi)
{
  DOIT();
  MultiThreadedSpinner ms;
  miniros::spin(ms); // will block until ROS shutdown
}

TEST(Spinners, multifail)
{
  DOIT();
  MultiThreadedSpinner ms;
  std::thread th([&ms]() {miniros::spin(ms); });
  miniros::WallDuration(0.1).sleep(); // wait for thread to be started

  SingleThreadedSpinner ss2;
  EXPECT_THROW(miniros::spin(ss2), std::runtime_error);
  EXPECT_THROW(ss2.spin(), std::runtime_error);

  // running another multi-threaded spinner is allowed
  MultiThreadedSpinner ms2;
  miniros::spin(ms2); // will block until ROS shutdown
  th.join();
}

TEST(Spinners, async)
{
  DOIT();
  AsyncSpinner as1(2);
  as1.start();

  // running another AsyncSpinner is allowed
  AsyncSpinner as2(2);
  as2.start();
  as2.stop();

  SingleThreadedSpinner ss;
  EXPECT_THROW(miniros::spin(ss), std::runtime_error);
  EXPECT_THROW(ss.spin(), std::runtime_error);

  // running a multi-threaded spinner is allowed
  MultiThreadedSpinner ms;
  miniros::spin(ms); // will block until ROS shutdown

  miniros::waitForShutdown();
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  argc_ = argc;
  argv_ = argv;
  return RUN_ALL_TESTS();
}
