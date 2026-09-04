#include <gtest/gtest.h>

#include <std_msgs/Empty.hxx>
#include <std_msgs/Header.hxx>
#include <std_msgs/String.hxx>
#include <std_msgs/Bool.hxx>
#include <std_msgs/UInt8MultiArray.hxx>
#include <geometry_msgs/Point.hxx>
#include <geometry_msgs/Pose.hxx>
#include <geometry_msgs/PoseStamped.hxx>
#include <geometry_msgs/Polygon.hxx>

#include "miniros/json_serialization.h"

using miniros::serialization::serializeJson;
using miniros::serialization::jsonCompact;
using miniros::serialization::jsonStrict;

TEST(JsonSerialization, EmptyMessage)
{
  std_msgs::Empty msg;
  EXPECT_EQ(serializeJson(msg), "{}");
}

TEST(JsonSerialization, StrictKeepsRosbridgeTimeKeys)
{
  std_msgs::Header msg;
  msg.stamp.sec = 1;
  msg.stamp.nsec = 2;
  EXPECT_NE(serializeJson(msg, jsonStrict).find("\"secs\":1"), std::string::npos);
  EXPECT_EQ(serializeJson(msg, jsonStrict).find("\"sec\":"), std::string::npos);
}

TEST(JsonSerialization, BoolIsJsonBoolean)
{
  std_msgs::Bool msg;
  msg.data = 1;
  EXPECT_EQ(serializeJson(msg), "{\"data\":true}");
  msg.data = 0;
  EXPECT_EQ(serializeJson(msg), "{\"data\":false}");
}

TEST(JsonSerialization, StringEscape)
{
  std_msgs::String msg;
  msg.data = "say \"hi\"\n\\";
  EXPECT_EQ(serializeJson(msg), "{\"data\":\"say \\\"hi\\\"\\n\\\\\"}");
}

TEST(JsonSerialization, NestedPose)
{
  geometry_msgs::Pose pose;
  pose.position.x = 1;
  pose.position.y = 2;
  pose.position.z = 3;
  pose.orientation.w = 1;
  const std::string json = serializeJson(pose);
  EXPECT_NE(json.find("\"position\":{"), std::string::npos);
  EXPECT_NE(json.find("\"orientation\":{"), std::string::npos);
  EXPECT_NE(json.find("\"x\":1"), std::string::npos);
  EXPECT_NE(json.find("\"w\":1"), std::string::npos);
}

TEST(JsonSerialization, PoseStamped)
{
  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = "base";
  msg.pose.position.x = 4.5;
  const std::string json = serializeJson(msg);
  EXPECT_NE(json.find("\"header\":{"), std::string::npos);
  EXPECT_NE(json.find("\"pose\":{"), std::string::npos);
  EXPECT_NE(json.find("\"frame_id\":\"base\""), std::string::npos);
  EXPECT_NE(json.find("\"secs\":"), std::string::npos);
}

TEST(JsonSerialization, PolygonArray)
{
  geometry_msgs::Polygon poly;
  geometry_msgs::Point32 a;
  a.x = 1;
  a.y = 0;
  a.z = 0;
  geometry_msgs::Point32 b;
  b.x = 0;
  b.y = 1;
  b.z = 0;
  poly.points.push_back(a);
  poly.points.push_back(b);
  EXPECT_EQ(serializeJson(poly),
            "{\"points\":[{\"x\":1,\"y\":0,\"z\":0},{\"x\":0,\"y\":1,\"z\":0}]}");
}

TEST(JsonSerialization, Uint8ArrayRosbridgeBase64)
{
  std_msgs::UInt8MultiArray msg;
  msg.data = {1, 2, 3};
  EXPECT_EQ(serializeJson(msg), "{\"layout\":{\"dim\":[],\"data_offset\":0},\"data\":\"AQID\"}");
}

TEST(JsonSerialization, Uint8ArrayStrictNumbers)
{
  std_msgs::UInt8MultiArray msg;
  msg.data = {1, 2, 3};
  EXPECT_EQ(serializeJson(msg, jsonStrict),
            "{\"layout\":{\"dim\":[],\"data_offset\":0},\"data\":[1,2,3]}");
}

TEST(JsonSerialization, Uint8ArrayBase64HasNoMimeNewlines)
{
  std_msgs::UInt8MultiArray msg;
  msg.data.assign(200, 0xff);
  const std::string json = serializeJson(msg);
  EXPECT_EQ(json.find('\n'), std::string::npos);
  EXPECT_NE(json.find("\"data\":\""), std::string::npos);
}

TEST(JsonSerialization, PrettyPrint)
{
  std_msgs::Header msg;
  msg.seq = 1;
  msg.frame_id = "map";
  miniros::JsonSettings pretty;
  pretty.tabs = 2;
  const std::string json = serializeJson(msg, pretty);
  EXPECT_NE(json.find('\n'), std::string::npos);
  EXPECT_NE(json.find("  \"seq\": 1"), std::string::npos);
}

TEST(JsonSerialization, AppendToExistingString)
{
  std_msgs::Empty msg;
  std::string out = "prefix";
  serializeJson(out, msg, jsonCompact);
  EXPECT_EQ(out, "prefix{}");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
