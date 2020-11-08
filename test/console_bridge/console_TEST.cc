#include <cassert>
#include <gtest/gtest.h>

#include <console_bridge/console.h>

//////////////////////////////////////////////////
TEST(ConsoleTest, MacroExpansionTest_ItShouldCompile)
{
  if (true)
    MINIROS_CONSOLE_BRIDGE_logDebug("Testing Log");

  if (true)
    MINIROS_CONSOLE_BRIDGE_logDebug("Testing Log");
  else
  {
      assert(true);
  }

  if (true)
  {
    MINIROS_CONSOLE_BRIDGE_logDebug("Testing Log");
  }
  else
  {
    MINIROS_CONSOLE_BRIDGE_logDebug("Testing Log");
  }
}
