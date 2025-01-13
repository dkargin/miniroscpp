// Publish big data chunks
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <miniros/publisher.h>
#include <miniros/init.h>
#include <miniros/node_handle.h>

#include <std_msgs/Int8MultiArray.h>

int main(int argc, char** argv)
{
  miniros::init(argc, argv, "publisher");
  miniros::NodeHandle n;

  const size_t NUM_BYTES = 8;
  std_msgs::Int8MultiArray data;
  data.data.reserve(NUM_BYTES);

  assert(argc > 1);
  float frequency = atof(argv[1]);

  miniros::Publisher pub = n.advertise<std_msgs::Int8MultiArray>("data", 1);
  miniros::Rate rate(frequency);

  size_t start = 0;
  while(miniros::ok())
  {
    data.data.clear();
    for(size_t i = 0; i < NUM_BYTES; ++i)
    {
      data.data.push_back(start + i);
    }
    pub.publish(data);
    rate.sleep();
    start++;
  }
  return 0;
}
