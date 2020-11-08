#include <iostream>
#include <nav_msgs/Odometry.hxx>

#include <minibag/bag.h>
#include <minibag/view.h>

int main(int argc, char* argv[])
{
  minibag::Bag bag;
  std::string path = argv[1];

  try
  {
    bag.open(path, minibag::bagmode::Write);
  }
  catch(minibag::BagException &ex)
  {
    std::cerr << "Failed to open bag " << path.c_str() << ": " << ex.what() << std::endl;
    return -1;
  }

  int maxRecords = 100;
  for (int i = 0; i < maxRecords; i++)
  {
      nav_msgs::Odometry odometry;
      odometry.header.frame_id = "odom";
      odometry.header.stamp = miniros::Time(i*0.1 + 0.1);
      bag.write("odom", odometry.header.stamp, odometry);
  }

  bag.close();
  return 0;
}
