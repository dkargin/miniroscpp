#include <miniros/ros.h>

int main(int argc, char **argv)
{ 
  miniros::init(argc, argv, "test_get_param");

  miniros::NodeHandle nh;
  nh.setParam(std::string("monkey"), false);
  bool test_bool;
  while(miniros::ok()) {
    if(!nh.getParam("monkey", test_bool)) {
      MINIROS_INFO_STREAM("Failed, bailing");
      miniros::shutdown();
    }
    std::cout << ".";
  }
  return 0;
}
