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

  int frames_publish = -1;

  int maxScans = 100;
  /// Number of parsed messages
  int numScans = 0, numOdoms = 0;

  minibag::View view(bag, minibag::TopicQuery(topics));
  int totalMsgs = view.size();
  int msgs;

  int frame = 0;

  for(minibag::MessageInstance const m: view)
  {
    if(frames_publish > 0 && frame >= frames_publish)
    {
      frame = 0;
      //test.publishMap();
      //test.publishTrajectory();
    }

    do
    {
      auto scan = m.instantiate<sensor_msgs::LaserScan>();
      if (scan != nullptr)
      {
        //test.onScan(*scan);
        numScans++;
        frame++;
        break;
      }
      auto odom = m.instantiate<nav_msgs::Odometry>();
      if (odom != nullptr)
      {
        //test.onScan(*scan);
        numOdoms++;
        frame++;
        break;
      }
    }while(false);

    if(numScans >= maxScans && maxScans > 0)
    {
      printf("Hit max scans = %d\n", numScans);
      break;
    }
  }

  //ROS_INFO("Processed %d scan and %d odometry readings", numScans, numOdoms);
  bag.close();
  return 0;
}
