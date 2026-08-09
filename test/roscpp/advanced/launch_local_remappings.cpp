//
// C++ stand-in for former launch/local_remappings.xml:
//   <param name="use_local_remap" type="bool" value="true" />
//   <param name="remap_from" value="sub_namespace"/>
//   <param name="remap_to"   value="different_sub_namespace" />
//   <test ... args="/base_namespace /base_namespace/different_sub_namespace" />
//

#include "launch_remapping.h"

int main()
{
  return miniros::test::launchTestRemapping(
      "launch_local_remappings",
      /*use_local_remap=*/true,
      {"/base_namespace", "/base_namespace/different_sub_namespace"},
      "sub_namespace",
      "different_sub_namespace");
}
