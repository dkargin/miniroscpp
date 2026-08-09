//
// C++ stand-in for former launch/global_remappings.xml:
//   <param name="use_local_remap" type="bool" value="false" />
//   <remap from="base_namespace" to="remapped_base_namespace" />
//   <remap from="remapped_base_namespace/sub_namespace"
//          to="remapped_base_namespace/remapped_sub_namespace" />
//   <test ... args="/remapped_base_namespace
//                    /remapped_base_namespace/remapped_sub_namespace"/>
//

#include "launch_remapping.h"

int main()
{
  return miniros::test::launchTestRemapping(
      "launch_global_remappings",
      /*use_local_remap=*/false,
      {"base_namespace:=remapped_base_namespace",
       "remapped_base_namespace/sub_namespace:=remapped_base_namespace/remapped_sub_namespace",
       "/remapped_base_namespace",
       "/remapped_base_namespace/remapped_sub_namespace"});
}
