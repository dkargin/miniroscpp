///////////////////////////////////////////////////////////////////////////////
// The roscpp_demo package has a few demos of the roscpp c++ client library 
//
// Copyright (C) 2008, Morgan Quigley
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice, 
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright 
//     notice, this list of conditions and the following disclaimer in the 
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its 
//     contributors may be used to endorse or promote products derived from 
//     this software without specific prior written permission.
//   
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

#include <time.h>

#include "miniros/ros.h"
#include <test_roscpp/TestArray.hxx>

void callback(const test_roscpp::TestArrayConstPtr&)
{
}

int main(int argc, char **argv)
{
  miniros::init(argc, argv, "subscribe_unsubscribe_repeatedly");
  miniros::NodeHandle nh;
  miniros::Duration sleep_time(0, 100000000);
  while(miniros::ok())
  {
    sleep_time.sleep();
    miniros::Subscriber sub = nh.subscribe("roscpp/pubsub_test", 1, callback);
    sleep_time.sleep();
  }
  
  return 0;
}
