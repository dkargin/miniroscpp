#include <gtest/gtest.h>

#include "miniros/utility/parameter_collection.h"
#include "miniros/utility/yaml.h"

using miniros::Error;
using miniros::ParameterCollection;
using miniros::RpcValue;
using miniros::parseYaml;

TEST(Yaml, FlatScalarsAndComments)
{
  const char* text = R"(
%YAML:1.0
---
# gravity magnitude
g_norm: 9.8154
windowSize: 21
useBarometer: 1
flowBack: false
name: PINHOLE
empty: ""
td: 0.0
gyr_w: 1.0e-4
)";
  RpcValue root;
  ASSERT_TRUE(parseYaml(text, root));
  ASSERT_EQ(root.getType(), RpcValue::TypeStruct);
  EXPECT_NEAR(static_cast<double>(root["g_norm"]), 9.8154, 1e-9);
  EXPECT_EQ(static_cast<int>(root["windowSize"]), 21);
  EXPECT_EQ(static_cast<int>(root["useBarometer"]), 1);
  EXPECT_FALSE(static_cast<bool>(root["flowBack"]));
  EXPECT_EQ(static_cast<const std::string&>(root["name"]), "PINHOLE");
  EXPECT_EQ(static_cast<const std::string&>(root["empty"]), "");
  EXPECT_NEAR(static_cast<double>(root["td"]), 0.0, 1e-12);
  EXPECT_NEAR(static_cast<double>(root["gyr_w"]), 1.0e-4, 1e-12);
}

TEST(Yaml, NestedMapAndFlowSeqWithSpaces)
{
  const char* text = R"(
objectDetector:
  backend: onnxruntime
  input_width: 640
  class_names: [car, moto, person, bus, construction equipment, train]
distortion_parameters:
   k1: -2.95e-01
   k2: 8.66e-02
)";
  RpcValue root;
  ASSERT_TRUE(parseYaml(text, root));
  EXPECT_EQ(static_cast<const std::string&>(root["objectDetector"]["backend"]), "onnxruntime");
  EXPECT_EQ(static_cast<int>(root["objectDetector"]["input_width"]), 640);
  const RpcValue& names = root["objectDetector"]["class_names"];
  ASSERT_TRUE(names.isArray());
  EXPECT_EQ(names.size(), 6);
  EXPECT_EQ(static_cast<const std::string&>(names[0]), "car");
  EXPECT_EQ(static_cast<const std::string&>(names[4]), "construction equipment");
  EXPECT_NEAR(static_cast<double>(root["distortion_parameters"]["k1"]), -0.295, 1e-6);
}

TEST(Yaml, BlockListOfMapsAndOpencvMatrix)
{
  const char* text = R"(
imus:
  - name: "imu0"
    topic: "/imu"
    acc_n: 0.1
    pose:
      t: [ 0.033, 0.0, 0.0 ]
  - name: "imu1"
    topic: "/msp"
body_T_cam0: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [1, 0, 0, 0,
          0, 1, 0, 0.537,
          0, 0, 1, 0,
          0, 0, 0, 1]
)";
  RpcValue root;
  ASSERT_TRUE(parseYaml(text, root));
  ASSERT_TRUE(root["imus"].isArray());
  EXPECT_EQ(root["imus"].size(), 2);
  EXPECT_EQ(static_cast<const std::string&>(root["imus"][0]["name"]), "imu0");
  EXPECT_EQ(static_cast<const std::string&>(root["imus"][0]["topic"]), "/imu");
  EXPECT_EQ(root["imus"][0]["pose"]["t"].size(), 3);
  EXPECT_NEAR(static_cast<double>(root["imus"][0]["pose"]["t"][0]), 0.033, 1e-9);
  EXPECT_EQ(static_cast<const std::string&>(root["imus"][1]["name"]), "imu1");

  EXPECT_EQ(static_cast<int>(root["body_T_cam0"]["rows"]), 4);
  EXPECT_EQ(static_cast<const std::string&>(root["body_T_cam0"]["dt"]), "d");
  EXPECT_EQ(root["body_T_cam0"]["data"].size(), 16);
  EXPECT_NEAR(static_cast<double>(root["body_T_cam0"]["data"][7]), 0.537, 1e-9);
}

TEST(Yaml, FlowMapAndNestedLists)
{
  const char* text = R"(
cam0:
  T_cam_imu:
    - [0.0, -1.0, 0.0, 0.02]
    - [1.0, 0.0, 0.0, 0.0]
    - [0.0, 0.0, 1.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
  cam_overlaps: []
nodes:
  - { nodeId:1, parentId:0, weight:0., descriptor:"52 233 120" }
)";
  RpcValue root;
  ASSERT_TRUE(parseYaml(text, root));
  EXPECT_EQ(root["cam0"]["T_cam_imu"].size(), 4);
  EXPECT_EQ(root["cam0"]["T_cam_imu"][0].size(), 4);
  EXPECT_NEAR(static_cast<double>(root["cam0"]["T_cam_imu"][0][3]), 0.02, 1e-9);
  EXPECT_TRUE(root["cam0"]["cam_overlaps"].isArray());
  EXPECT_EQ(root["cam0"]["cam_overlaps"].size(), 0);
  EXPECT_EQ(static_cast<int>(root["nodes"][0]["nodeId"]), 1);
  EXPECT_EQ(static_cast<const std::string&>(root["nodes"][0]["descriptor"]), "52 233 120");
}

TEST(Yaml, ParameterCollectionRoundTrip)
{
  ParameterCollection params;
  params.addBool("enabled", true).description("Most important bool");
  params.addInt("width", 1920);
  params.addDouble("td", 0.01);
  params.addEnum("format", {{"YUYV", "YUYV"}, {"MJPEG", "MJPEG"}}, "YUYV")
      .description("Pixel format");

  ParameterCollection loaded;
  loaded.addBool("enabled", false);
  loaded.addInt("width", 0);
  loaded.addDouble("td", 0.0);
  loaded.addEnum("format", {{"YUYV", "YUYV"}, {"MJPEG", "MJPEG"}}, "MJPEG");
  ASSERT_TRUE(loaded.loadYaml(params.toYaml("grabber")));
  EXPECT_TRUE(loaded.getBool("enabled"));
  EXPECT_EQ(loaded.getInt("width"), 1920);
  EXPECT_NEAR(loaded.getDouble("td"), 0.01, 1e-12);
  EXPECT_EQ(loaded.getString("format"), "YUYV");
}

TEST(Yaml, LoadMissingFile)
{
  RpcValue root;
  EXPECT_EQ(miniros::loadYamlFile("/no/such/grabber.yaml", root).code, Error::FileNotFound);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
