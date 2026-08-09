//
// C++ stand-in for former launch/no_remappings.xml:
//   <param name="use_local_remap" type="bool" value="false" />
//   <test ... args="/base_namespace /base_namespace/sub_namespace"/>
//

#include "launch_remapping.h"

int main()
{
  return miniros::test::launchTestRemapping(
      "launch_no_remappings",
      /*use_local_remap=*/false,
      {"/base_namespace", "/base_namespace/sub_namespace"});
}
