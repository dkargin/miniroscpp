#include <map>
#include <string>

#include <miniros/ros.h>
#include <gtest/gtest.h>
#include <rosgraph_msgs/TopicStatistics.h>
#include <boost/thread.hpp>
#include <std_msgs/Int8MultiArray.h>

class Aggregator {
public:
  std::map<std::string, miniros::Duration> topic_period_mean_map_;

  void cb(const rosgraph_msgs::TopicStatistics& msg) {
    topic_period_mean_map_[msg.topic] = msg.period_mean;
  }

  bool frequencyAcceptable(const std::string& topic, float expected) {
    float errorMargin = 0.1;
    float foundFreq = 1.f / topic_period_mean_map_[topic].toSec();
    return std::fabs(foundFreq - expected) / expected <= errorMargin;
  }
};

void assertEventuallyHasTopic(const Aggregator& agg, const std::string& topic) {
  miniros::Duration timeout(5.0);
  auto start = miniros::Time::now();
  while (miniros::Time::now() - start < timeout && !agg.topic_period_mean_map_.count(topic)) {
    miniros::Duration(0.5).sleep();
  }
  ASSERT_EQ(agg.topic_period_mean_map_.count(topic), 1u);
}

TEST(TopicStatisticFrequency, statisticFrequency)
{
  miniros::NodeHandle nh;
  Aggregator agg;
  miniros::Subscriber stat_sub = nh.subscribe("/statistics", 1, &Aggregator::cb, &agg);

  miniros::AsyncSpinner spinner(4);
  spinner.start();

  miniros::Duration(5.0).sleep();

  assertEventuallyHasTopic(agg, "/very_fast_chatter");
  assertEventuallyHasTopic(agg, "/fast_chatter");
  assertEventuallyHasTopic(agg, "/slow_chatter");
  assertEventuallyHasTopic(agg, "/very_slow_chatter");

  miniros::shutdown();

  ASSERT_TRUE(agg.frequencyAcceptable("/very_fast_chatter", 171));
  ASSERT_TRUE(agg.frequencyAcceptable("/fast_chatter", 53));
  ASSERT_TRUE(agg.frequencyAcceptable("/slow_chatter", 18));
  ASSERT_TRUE(agg.frequencyAcceptable("/very_slow_chatter", 0.8));
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  miniros::init(argc, argv, "topic_statistic_frequency");
  miniros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
