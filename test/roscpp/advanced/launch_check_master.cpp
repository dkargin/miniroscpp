//
// C++ stand-in for former launch/check_master.xml:
//   <test ... type="advanced-check_master" args="yes"/>
//

#include "launch_child.h"

int main()
{
  return miniros::test::launchTestChild(
      "launch_check_master",
      "advanced-check_master",
      {"yes"});
}
