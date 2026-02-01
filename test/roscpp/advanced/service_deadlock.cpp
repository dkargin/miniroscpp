#include <cstdlib>
#include <gtest/gtest.h>

#include "miniros/ros.h"
#include "std_srvs/Empty.hxx"
#include <miniros/console.h>
#include <miniros/io/poll_manager.h>

bool dummyService(std_srvs::Empty::Request &, std_srvs::Empty::Request &)
{
  return true;
}

static const char SERVICE1[] = "service1";

void call(miniros::ServiceClient &client)
{
  if (client && client.exists() && client.isValid())
  {
    MINIROS_INFO("Calling service");
    std_srvs::Empty srv;
    // these will alternate successful and failed.
    if (client.call(srv))
      MINIROS_INFO("  Successful call");
    else
      MINIROS_INFO("  Failed to call");
  }
  else
    MINIROS_INFO("Persistent client is invalid");
}

// this only verifies that it doesn't deadlock.  Should run about 60 seconds.
TEST(roscpp, ServiceDeadlocking)
{
  miniros::ServiceClient client;
  miniros::AsyncSpinner spinner(3);
  spinner.start();

  unsigned j = 0;
  miniros::Time start_time = miniros::Time::now();
  unsigned seconds = 30;
  miniros::Time stop_time = start_time + miniros::Duration(seconds, 0);

  while (true)
  {
    if ((j % 500 == 0) && (miniros::Time::now() > stop_time))
      break;

    {
      miniros::NodeHandle n2;
      miniros::ServiceServer service = n2.advertiseService(SERVICE1, dummyService);
      client  = n2.serviceClient<std_srvs::Empty>(SERVICE1, true);
      call(client);
      service.shutdown();
    }
    miniros::NodeHandle n;
    miniros::ServiceServer service = n.advertiseService(SERVICE1, dummyService);

    call(client);
    ++j;
  }
  MINIROS_INFO("Made it through %u loops in %u seconds", j, seconds);
  ASSERT_GE(j, 1000u);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  miniros::init(argc, argv, "service_deadlock");
  return RUN_ALL_TESTS();
}
