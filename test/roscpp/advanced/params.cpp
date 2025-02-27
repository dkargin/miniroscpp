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
 * Test parameters
 */

#include <string>
#include <sstream>
#include <fstream>

#include <gtest/gtest.h>

#include <time.h>
#include <stdlib.h>

#include "miniros/ros.h"

#include "master_fixture.h"
#include "miniros/xmlrpcpp/XmlRpcValue.h"

using namespace miniros;

TEST_F(MasterFixture, allParamTypes)
{
  std::string string_param;
  EXPECT_TRUE( master->get( "string", string_param ) );
  EXPECT_TRUE( string_param == "test" );

  int int_param = 0;
  EXPECT_TRUE( master->get( "int", int_param ) );
  EXPECT_TRUE( int_param == 10 );

  double double_param = 0.0;
  EXPECT_TRUE( master->get( "double", double_param ) );
  EXPECT_DOUBLE_EQ( double_param, 10.5 );

  bool bool_param = true;
  EXPECT_TRUE( master->get( "bool", bool_param ) );
  EXPECT_FALSE( bool_param );
}

TEST_F(MasterFixture, setThenGetString)
{
  master->set( "test_set_param", std::string("asdf") );
  std::string param;
  ASSERT_TRUE( master->get( "test_set_param", param ) );
  ASSERT_STREQ( "asdf", param.c_str() );
  
  XmlRpc::XmlRpcValue v;
  master->get("test_set_param", v);
  ASSERT_EQ(v.getType(), XmlRpc::XmlRpcValue::TypeString);
}

TEST_F(MasterFixture, setThenGetStringCached)
{
  std::string param;
  ASSERT_FALSE( master->getCached( "test_set_param_setThenGetStringCached", param) );

  master->set( "test_set_param_setThenGetStringCached", std::string("asdf") );

  ASSERT_TRUE( master->getCached( "test_set_param_setThenGetStringCached", param) );
  ASSERT_STREQ( "asdf", param.c_str() );
}

TEST_F(MasterFixture, setThenGetStringCachedNodeHandle)
{
  NodeHandle nh;
  std::string param;
  ASSERT_FALSE( nh.getParamCached( "test_set_param_setThenGetStringCachedNodeHandle", param) );

  nh.setParam( "test_set_param_setThenGetStringCachedNodeHandle", std::string("asdf") );

  ASSERT_TRUE( nh.getParamCached( "test_set_param_setThenGetStringCachedNodeHandle", param) );
  ASSERT_STREQ( "asdf", param.c_str() );
}

TEST_F(MasterFixture, setThenGetNamespaceCached)
{
  std::string stringParam;
  XmlRpc::XmlRpcValue structParam;
  const std::string ns = "test_set_param_setThenGetStringCached2";
  ASSERT_FALSE(master->getCached(ns, stringParam));

  master->set(ns, std::string("a"));
  ASSERT_TRUE(master->getCached(ns, stringParam));
  ASSERT_STREQ("a", stringParam.c_str());

  master->set(ns + "/foo", std::string("b"));
  ASSERT_TRUE(master->getCached(ns + "/foo", stringParam));
  ASSERT_STREQ("b", stringParam.c_str());
  ASSERT_TRUE(master->getCached(ns, structParam));
  ASSERT_TRUE(structParam.hasMember("foo"));
  ASSERT_STREQ("b", static_cast<std::string>(structParam["foo"]).c_str());
}

TEST_F(MasterFixture, setThenGetCString)
{
  master->set( "test_set_param", "asdf" );
  std::string param;
  ASSERT_TRUE( master->get( "test_set_param", param ) );
  ASSERT_STREQ( "asdf", param.c_str() );
}

TEST_F(MasterFixture, setThenGetInt)
{
  master->set( "test_set_param", 42);
  int param;
  ASSERT_TRUE( master->get( "test_set_param", param ) );
  ASSERT_EQ( 42, param );
  XmlRpc::XmlRpcValue v;
  master->get("test_set_param", v);
  ASSERT_EQ(v.getType(), XmlRpc::XmlRpcValue::TypeInt);
}

TEST_F(MasterFixture, unknownParam)
{
  std::string param;
  ASSERT_FALSE( master->get( "this_param_really_should_not_exist", param ) );
}

TEST_F(MasterFixture, deleteParam)
{
  master->set( "test_delete_param", "asdf" );
  master->del( "test_delete_param" );
  std::string param;
  ASSERT_FALSE( master->get( "test_delete_param", param ) );
}

TEST_F(MasterFixture, hasParam)
{
  ASSERT_TRUE( master->has( "string" ) );
}

TEST_F(MasterFixture, setIntDoubleGetInt)
{
  master->set("test_set_int_as_double", 1);
  master->set("test_set_int_as_double", 3.0f);

  int i = -1;
  ASSERT_TRUE(master->get("test_set_int_as_double", i));
  ASSERT_EQ(3, i);
  double d = 0.0f;
  ASSERT_TRUE(master->get("test_set_int_as_double", d));
  ASSERT_EQ(3.0, d);
}

TEST_F(MasterFixture, getIntAsDouble)
{
  master->set("int_param", 1);
  double d = 0.0;
  ASSERT_TRUE(master->get("int_param", d));
  ASSERT_EQ(1.0, d);
}

TEST_F(MasterFixture, getDoubleAsInt)
{
  master->set("double_param", 2.3);
  int i = -1;
  ASSERT_TRUE(master->get("double_param", i));
  ASSERT_EQ(2, i);

  master->set("double_param", 3.8);
  i = -1;
  ASSERT_TRUE(master->get("double_param", i));
  ASSERT_EQ(4, i);
}

TEST_F(MasterFixture, searchParam)
{
  std::string ns = "/a/b/c/d/e/f";
  std::string result;

  master->set("/s_i", 1);
  ASSERT_TRUE(master->search(ns, "s_i", result));
  ASSERT_STREQ(result.c_str(), "/s_i");
  master->del("/s_i");

  master->set("/a/b/s_i", 1);
  ASSERT_TRUE(master->search(ns, "s_i", result));
  ASSERT_STREQ(result.c_str(), "/a/b/s_i");
  master->del("/a/b/s_i");

  master->set("/a/b/c/d/e/f/s_i", 1);
  ASSERT_TRUE(master->search(ns, "s_i", result));
  ASSERT_STREQ(result.c_str(), "/a/b/c/d/e/f/s_i");
  master->del("/a/b/c/d/e/f/s_i");

  bool cont = true;
  while (!cont)
  {
    miniros::WallDuration(0.1).sleep();
  }

  ASSERT_FALSE(master->search(ns, "s_j", result));
}

TEST_F(MasterFixture, searchParamNodeHandle)
{
  NodeHandle n("/a/b/c/d/e/f");
  std::string result;

  n.setParam("/s_i", 1);
  ASSERT_TRUE(n.searchParam("s_i", result));
  ASSERT_STREQ(result.c_str(), "/s_i");
  n.deleteParam("/s_i");

  n.setParam("/a/b/s_i", 1);
  ASSERT_TRUE(n.searchParam("s_i", result));
  ASSERT_STREQ(result.c_str(), "/a/b/s_i");
  n.deleteParam("/a/b/s_i");

  n.setParam("/a/b/c/d/e/f/s_i", 1);
  ASSERT_TRUE(n.searchParam("s_i", result));
  ASSERT_STREQ(result.c_str(), "/a/b/c/d/e/f/s_i");
  n.deleteParam("/a/b/c/d/e/f/s_i");

  ASSERT_FALSE(n.searchParam("s_j", result));
}

TEST_F(MasterFixture, searchParamNodeHandleWithRemapping)
{
  M_string remappings;
  remappings["s_c"] = "s_b";
  NodeHandle n("/a/b/c/d/e/f", remappings);
  std::string result;

  n.setParam("/s_c", 1);
  ASSERT_FALSE(n.searchParam("s_c", result));
  n.setParam("/s_b", 1);
  ASSERT_TRUE(n.searchParam("s_c", result));
}

// See ROS ticket #2381
TEST_F(MasterFixture, getMissingXmlRpcValueParameterCachedTwice)
{
  XmlRpc::XmlRpcValue v;
  ASSERT_FALSE(master->getCached("invalid_xmlrpcvalue_param", v));
  ASSERT_FALSE(master->getCached("invalid_xmlrpcvalue_param", v));
}

// See ROS ticket #2353
TEST_F(MasterFixture, doublePrecision)
{
  master->set("bar", 0.123456789123456789);
  double d;
  ASSERT_TRUE(master->get("bar", d));
  EXPECT_DOUBLE_EQ(d, 0.12345678912345678);
}

std::vector<std::string> vec_s, vec_s2;
std::vector<double> vec_d, vec_d2;
std::vector<float> vec_f, vec_f2;
std::vector<int> vec_i, vec_i2;
std::vector<bool> vec_b, vec_b2;

TEST_F(MasterFixture, vectorStringParam)
{
  const std::string param_name = "vec_str_param";

  vec_s.clear();
  vec_s.push_back("foo");
  vec_s.push_back("bar");
  vec_s.push_back("baz");

  master->set(param_name, vec_s);

  ASSERT_FALSE(master->get(param_name, vec_d));
  ASSERT_FALSE(master->get(param_name, vec_f));
  ASSERT_FALSE(master->get(param_name, vec_i));
  ASSERT_FALSE(master->get(param_name, vec_b));

  ASSERT_TRUE(master->get(param_name, vec_s2));

  ASSERT_EQ(vec_s.size(), vec_s2.size());
  ASSERT_TRUE(std::equal(vec_s.begin(), vec_s.end(), vec_s2.begin()));

  // Test empty vector
  vec_s.clear();
  master->set(param_name, vec_s);
  ASSERT_TRUE(master->get(param_name, vec_s2));
  ASSERT_EQ(vec_s.size(), vec_s2.size());
}

TEST_F(MasterFixture, vectorDoubleParam)
{
  const std::string param_name = "vec_double_param";

  vec_d.clear();
  vec_d.push_back(-0.123456789);
  vec_d.push_back(3);
  vec_d.push_back(3.01);
  vec_d.push_back(7.01);

  master->set(param_name, vec_d);

  ASSERT_FALSE(master->get(param_name, vec_s));
  ASSERT_TRUE(master->get(param_name, vec_i));
  ASSERT_TRUE(master->get(param_name, vec_b));
  ASSERT_TRUE(master->get(param_name, vec_f));

  ASSERT_TRUE(master->get(param_name, vec_d2));

  ASSERT_EQ(vec_d.size(), vec_d2.size());
  ASSERT_TRUE(std::equal(vec_d.begin(), vec_d.end(), vec_d2.begin()));
}

TEST_F(MasterFixture, vectorFloatParam)
{
  const std::string param_name = "vec_float_param";

  vec_f.clear();
  vec_f.push_back(-0.123456789);
  vec_f.push_back(0.0);
  vec_f.push_back(3);
  vec_f.push_back(3.01);

  master->set(param_name, vec_f);

  ASSERT_FALSE(master->get(param_name, vec_s));
  ASSERT_TRUE(master->get(param_name, vec_i));
  ASSERT_TRUE(master->get(param_name, vec_b));
  ASSERT_TRUE(master->get(param_name, vec_d));

  ASSERT_EQ(vec_b[0],true);
  ASSERT_EQ(vec_b[1],false);

  ASSERT_TRUE(master->get(param_name, vec_f2));

  ASSERT_EQ(vec_f.size(), vec_f2.size());
  ASSERT_TRUE(std::equal(vec_f.begin(), vec_f.end(), vec_f2.begin()));
}

TEST_F(MasterFixture, vectorIntParam)
{
  const std::string param_name = "vec_int_param";

  vec_i.clear();
  vec_i.push_back(-1);
  vec_i.push_back(0);
  vec_i.push_back(1337);
  vec_i.push_back(2);

  master->set(param_name, vec_i);

  ASSERT_FALSE(master->get(param_name, vec_s));
  ASSERT_TRUE(master->get(param_name, vec_d));
  ASSERT_TRUE(master->get(param_name, vec_f));
  ASSERT_TRUE(master->get(param_name, vec_b));

  ASSERT_EQ(vec_b[0],true);
  ASSERT_EQ(vec_b[1],false);

  ASSERT_TRUE(master->get(param_name, vec_i2));

  ASSERT_EQ(vec_i.size(), vec_i2.size());
  ASSERT_TRUE(std::equal(vec_i.begin(), vec_i.end(), vec_i2.begin()));
}

TEST_F(MasterFixture, vectorBoolParam)
{
  const std::string param_name = "vec_bool_param";

  vec_b.clear();
  vec_b.push_back(true);
  vec_b.push_back(false);
  vec_b.push_back(true);
  vec_b.push_back(true);

  master->set(param_name, vec_b);

  ASSERT_FALSE(master->get(param_name, vec_s));
  ASSERT_TRUE(master->get(param_name, vec_d));
  ASSERT_TRUE(master->get(param_name, vec_f));
  ASSERT_TRUE(master->get(param_name, vec_i));

  ASSERT_EQ(vec_i[0],1);
  ASSERT_EQ(vec_i[1],0);

  ASSERT_TRUE(master->get(param_name, vec_b2));

  ASSERT_EQ(vec_b.size(), vec_b2.size());
  ASSERT_TRUE(std::equal(vec_b.begin(), vec_b.end(), vec_b2.begin()));
}

std::map<std::string,std::string> map_s, map_s2;
std::map<std::string,double> map_d, map_d2;
std::map<std::string,float> map_f, map_f2;
std::map<std::string,int> map_i, map_i2;
std::map<std::string,bool> map_b, map_b2;

TEST_F(MasterFixture, mapStringParam)
{
  const std::string param_name = "map_str_param";

  map_s.clear();
  map_s["a"] = "apple";
  map_s["b"] = "blueberry";
  map_s["c"] = "carrot";

  master->set(param_name, map_s);

  ASSERT_FALSE(master->get(param_name, map_d));
  ASSERT_FALSE(master->get(param_name, map_f));
  ASSERT_FALSE(master->get(param_name, map_i));
  ASSERT_FALSE(master->get(param_name, map_b));

  ASSERT_TRUE(master->get(param_name, map_s2));

  ASSERT_EQ(map_s.size(), map_s2.size());
  ASSERT_TRUE(std::equal(map_s.begin(), map_s.end(), map_s2.begin()));
}

TEST_F(MasterFixture, mapDoubleParam)
{
  const std::string param_name = "map_double_param";

  map_d.clear();
  map_d["a"] = 0.0;
  map_d["b"] = -0.123456789;
  map_d["c"] = 123456789;

  master->set(param_name, map_d);

  ASSERT_FALSE(master->get(param_name, map_s));
  ASSERT_TRUE(master->get(param_name, map_f));
  ASSERT_TRUE(master->get(param_name, map_i));
  ASSERT_TRUE(master->get(param_name, map_b));

  ASSERT_TRUE(master->get(param_name, map_d2));

  ASSERT_EQ(map_d.size(), map_d2.size());
  ASSERT_TRUE(std::equal(map_d.begin(), map_d.end(), map_d2.begin()));
}

TEST_F(MasterFixture, mapFloatParam)
{
  const std::string param_name = "map_float_param";

  map_f.clear();
  map_f["a"] = 0.0;
  map_f["b"] = -0.123456789;
  map_f["c"] = 123456789;

  master->set(param_name, map_f);

  ASSERT_FALSE(master->get(param_name, map_s));
  ASSERT_TRUE(master->get(param_name, map_d));
  ASSERT_TRUE(master->get(param_name, map_i));
  ASSERT_TRUE(master->get(param_name, map_b));

  ASSERT_TRUE(master->get(param_name, map_f2));

  ASSERT_EQ(map_f.size(), map_f2.size());
  ASSERT_TRUE(std::equal(map_f.begin(), map_f.end(), map_f2.begin()));
}

TEST_F(MasterFixture, mapIntParam)
{
  const std::string param_name = "map_int_param";

  map_i.clear();
  map_i["a"] = 0;
  map_i["b"] = -1;
  map_i["c"] = 1337;

  master->set(param_name, map_i);

  ASSERT_FALSE(master->get(param_name, map_s));
  ASSERT_TRUE(master->get(param_name, map_d));
  ASSERT_TRUE(master->get(param_name, map_f));
  ASSERT_TRUE(master->get(param_name, map_b));

  ASSERT_TRUE(master->get(param_name, map_i2));

  ASSERT_EQ(map_i.size(), map_i2.size());
  ASSERT_TRUE(std::equal(map_i.begin(), map_i.end(), map_i2.begin()));
}

TEST_F(MasterFixture, mapBoolParam)
{
  const std::string param_name = "map_bool_param";

  map_b.clear();
  map_b["a"] = true;
  map_b["b"] = false;
  map_b["c"] = true;

  master->set(param_name, map_b);

  ASSERT_FALSE(master->get(param_name, map_s));
  ASSERT_TRUE(master->get(param_name, map_d));
  ASSERT_TRUE(master->get(param_name, map_f));
  ASSERT_TRUE(master->get(param_name, map_i));

  ASSERT_EQ(map_i["a"],1);
  ASSERT_EQ(map_i["b"],0);

  ASSERT_TRUE(master->get(param_name, map_b2));

  ASSERT_EQ(map_b.size(), map_b2.size());
  ASSERT_TRUE(std::equal(map_b.begin(), map_b.end(), map_b2.begin()));
}

TEST_F(MasterFixture, paramTemplateFunction)
{
  EXPECT_EQ( master->param<std::string>( "string", "" ), "test" );
  EXPECT_EQ( master->param<std::string>( "gnirts", "test" ), "test" );

  EXPECT_EQ( master->param<int>( "int", 0 ), 10 );
  EXPECT_EQ( master->param<int>( "tni", 10 ), 10 );

  EXPECT_DOUBLE_EQ( master->param<double>( "double", 0.0 ), 10.5 );
  EXPECT_DOUBLE_EQ( master->param<double>( "elbuod", 10.5 ), 10.5 );

  EXPECT_EQ( master->param<bool>( "bool", true ), false );
  EXPECT_EQ( master->param<bool>( "loob", true ), true );
}

TEST_F(MasterFixture, paramNodeHandleTemplateFunction)
{
  NodeHandle nh;

  EXPECT_EQ( nh.param<std::string>( "string", "" ), "test" );
  EXPECT_EQ( nh.param<std::string>( "gnirts", "test" ), "test" );

  EXPECT_EQ( nh.param<int>( "int", 0 ), 10 );
  EXPECT_EQ( nh.param<int>( "tni", 10 ), 10 );

  EXPECT_DOUBLE_EQ( nh.param<double>( "double", 0.0 ), 10.5 );
  EXPECT_DOUBLE_EQ( nh.param<double>( "elbuod", 10.5 ), 10.5 );

  EXPECT_EQ( nh.param<bool>( "bool", true ), false );
  EXPECT_EQ( nh.param<bool>( "loob", true ), true );
}

TEST_F(MasterFixture, getParamNames) {
  std::vector<std::string> test_params;
  EXPECT_TRUE(master->getParamNames(test_params));
  EXPECT_LT(10u, test_params.size());
}

TEST(Params, getParamCachedSetParamLoop) {
  NodeHandle nh;
  const std::string name = "changeable_int";
  for (int i = 0; i < 100; i++) {
    nh.setParam(name, i);
    int v = 0;
    ASSERT_TRUE(nh.getParamCached(name, v));
    ASSERT_EQ(i, v);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  miniros::init(argc, argv, "params");

  // These parameters were set by a rostest/roslaunch.
  // We set these manually to mimic behaviour of original test without roslaunch.
  auto master = miniros::getMasterLink();
  master->set("string", "test");
  master->set("int", 10);
  master->set("double", 10.5);
  master->set("bool", false);

  miniros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
