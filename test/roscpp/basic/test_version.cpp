/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Josh Faust */

/*
 * Test version macros
 */

#include <cstring>
#include <gtest/gtest.h>

#include "miniros/build_config.h"
#include "miniros/version.h"

TEST(version, minimum)
{
  ASSERT_TRUE(MINIROS_VERSION_GE(0, 5, 0, 0, 4, 0));
  ASSERT_TRUE(MINIROS_VERSION_GE(0, 5, 0, 0, 4, 10));
  ASSERT_FALSE(MINIROS_VERSION_GE(0, 5, 0, 999, 0, 0));
  ASSERT_FALSE(MINIROS_VERSION_GE(0, 5, 0, 0, 999, 0));
  ASSERT_TRUE(MINIROS_VERSION_GE(1, 0, 0, 0, 5, 0));
  ASSERT_TRUE(MINIROS_VERSION_GE(1, 0, 0, 0, 5, 1));
  ASSERT_FALSE(MINIROS_VERSION_GE(1, 0, 0, 2, 0, 0));
  ASSERT_FALSE(MINIROS_VERSION_GE(1, 1, 0, 2, 0, 0));
  ASSERT_FALSE(MINIROS_VERSION_GE(1, 0, 1, 2, 0, 0));

#if MINIROS_VERSION_GE(0, 5, 0, 999, 0, 0)
  FAIL();
#endif

#if !(MINIROS_VERSION_GE(0, 5, 0, 0, 4, 0))
  FAIL();
#endif
}

TEST(version, buildConfig)
{
  ASSERT_STREQ(MINIROS_VERSION_STRING, miniros::BuildConfig::getApiVersionString());
  ASSERT_STREQ(miniros::BuildConfig::getApiVersionString(),
               miniros::BuildConfig::getBinaryVersionString());
  ASSERT_STRNE("", miniros::BuildConfig::getApiVersionString());

  ASSERT_NE(miniros::BuildConfig::getGitCommit(), nullptr);
  ASSERT_NE(miniros::BuildConfig::getGitBranch(), nullptr);

  const char* commit = miniros::BuildConfig::getGitCommit();
  if (commit[0] != '\0') {
    ASSERT_GE(std::strlen(commit), 7u);
    ASSERT_STRNE("", miniros::BuildConfig::getGitBranch());
  }

  // ASan and TSan cannot be enabled together.
  ASSERT_FALSE(miniros::BuildConfig::useASan() && miniros::BuildConfig::useTSan());
}

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

