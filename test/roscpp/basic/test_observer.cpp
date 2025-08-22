/* Author: Dmitry Kargin dmitry.n.kargin@gmail.com, dv_frost@mail.ru */
/*
 * Test Observer pattern operation
 */

#include <gtest/gtest.h>
#include <miniros/internal/observer.h>

using namespace miniros::observer;

class Listener : public Connection {

};

class ObservedObject : public Target<Listener> {

};

TEST(Observers, ProperDetach)
{
  ObservedObject observed;

  {
    Listener listener1;
    Listener listener2;

    observed.attach(&listener1);
    observed.attach(&listener2);

    listener1.disconnect();
    EXPECT_FALSE(listener1.connected());
    EXPECT_EQ(listener1.next(), nullptr);
    EXPECT_EQ(listener1.prev(), nullptr);

    {
      Listener listener3;
      observed.attach(&listener3);
    }
  }
  EXPECT_FALSE(observed.hasConnections());
}

TEST(Observers, Destructor)
{
  Listener listener1;
  Listener listener2;

  {
    ObservedObject observed;
    observed.attach(&listener1);
    observed.attach(&listener2);
  }

  EXPECT_FALSE(listener1.connected());
  EXPECT_EQ(listener1.next(), nullptr);
  EXPECT_EQ(listener1.prev(), nullptr);

  EXPECT_FALSE(listener2.connected());
  EXPECT_EQ(listener2.next(), nullptr);
  EXPECT_EQ(listener2.prev(), nullptr);
}


TEST(Observers, DetachAll)
{
	Listener listener1;
	Listener listener2;

  ObservedObject observed;
  observed.attach(&listener1);
  observed.attach(&listener2);

  size_t disconnected = observed.disconnectAll();
  EXPECT_EQ(disconnected, 2);
  EXPECT_FALSE(observed.hasConnections());

	EXPECT_FALSE(listener1.connected());
	EXPECT_EQ(listener1.next(), nullptr);
	EXPECT_EQ(listener1.prev(), nullptr);

	EXPECT_FALSE(listener2.connected());
	EXPECT_EQ(listener2.next(), nullptr);
	EXPECT_EQ(listener2.prev(), nullptr);
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}


