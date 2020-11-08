#include <nav_msgs/Odometry.hxx>
#include <sensor_msgs/LaserScan.hxx>
#include <minibag/bag.h>
#include <minibag/view.h>

int main(int argc, char* argv[])
{
  minibag::Bag bag;
  std::string path = argv[1];

  try
  {
    bag.open(path, minibag::bagmode::Read);
  }
  catch(minibag::BagException &ex)
  {
    std::cerr << "Failed to open bag " << path.c_str() << ": " << ex.what() << std::endl;
    return -1;
  }

  std::vector<std::string> topics = {"scan", "odom"};

  /// Number of parsed messages
  int numScans = 0, numOdoms = 0;

  minibag::TopicQuery query(topics);
  minibag::View view(bag, query);
  int totalMsgs = view.size();
  std::cout << "Bag \"" << path.c_str() << " has " << totalMsgs << " messages" << std::endl;

  for(minibag::MessageInstance const m: view)
  {
    do
    {
      if (auto scan = m.instantiate<sensor_msgs::LaserScan>())
      {
        std::cout
          << "Got scan time=" << scan->header.stamp.toSec()
          << " frame_id=" << scan->header.frame_id.c_str() << std::endl;
        numScans++;
        break;
      }

      if (auto odom = m.instantiate<nav_msgs::Odometry>())
      {
        std::cout
            << "Got odometry time=" << odom->header.stamp.toSec()
            << " frame_id=" << odom->header.frame_id.c_str() << std::endl;
        numOdoms++;
        break;
      }
    }while(false);
  }

  std::cout << "Processed " << numScans << " scan and " << numOdoms << " odometry readings" << std::endl;
  bag.close();
  return 0;
}
