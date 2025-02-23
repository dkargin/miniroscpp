//
// Created by dkargin on 2/14/25.
//

#ifndef MASTER_FIXTURE_H
#define MASTER_FIXTURE_H

#include <miniros/ros.h>
#include <miniros/master_link.h>

#include <gtest/gtest.h>


class MasterFixture : public testing::Test
{
protected:
  miniros::MasterLinkPtr master;

  void SetUp() override
  {
    master = miniros::getMasterLink();
  }
};


#endif //MASTER_FIXTURE_H
