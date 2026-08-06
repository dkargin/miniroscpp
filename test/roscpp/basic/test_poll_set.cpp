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

#include <gtest/gtest.h>
#include "miniros/io/poll_set.h"
#include "miniros/io/io.h"

#include <atomic>
#include <thread>
#include <chrono>
#include <cassert>

#include "barrier.h"

using namespace miniros;

// PollSet still takes int fds; sockets are socket_fd_t (SOCKET on Win64).
static int poll_fd(socket_fd_t sock)
{
  return static_cast<int>(sock);
}

class Poller : public testing::Test
{
public:
  void waitThenSignal()
  {
    std::this_thread::sleep_for(std::chrono::microseconds(100000));

    poll_set_.signal();
  }

protected:

  void SetUp() override
  {
    if (create_socket_pair(sockets_) != 0)
    {
      FAIL();
    }
    if (set_non_blocking(sockets_[0]) != 0)
    {
      FAIL();
    }
    if (set_non_blocking(sockets_[1]) != 0)
    {
      FAIL();
    }
  }

  void TearDown() override
  {
    close_socket(sockets_[0]);
    close_socket(sockets_[1]);
  }

  PollSet poll_set_;
  socket_fd_t sockets_[2] = {MINIROS_INVALID_SOCKET, MINIROS_INVALID_SOCKET};
};

class SocketHelper
{
public:
  SocketHelper(socket_fd_t sock)
  : bytes_read_(0)
  , bytes_written_(0)
  , pollouts_received_(0)
  , socket_(sock)
  {}

  int fd() const { return poll_fd(socket_); }

  void processEvents(int events)
  {
    if (events & POLLIN)
    {
      char b;
      while (read_socket(socket_, &b, 1) > 0)
      {
        ++bytes_read_;
      };
    }

    if (events & POLLOUT)
    {
      ++pollouts_received_;

      write();
    }
  }

  void write()
  {
    char b = 0;
    if (write_socket(socket_, &b, 1) > 0)
    {
      ++bytes_written_;
    }
  }

  int bytes_read_;
  int bytes_written_;
  int pollouts_received_;
  socket_fd_t socket_;
};

static int write_byte(socket_fd_t sock)
{
  char b = 0;
  return write_socket(sock, &b, 1);
}

TEST_F(Poller, read)
{
  SocketHelper sh(sockets_[0]);
  ASSERT_TRUE(poll_set_.addSocket(sh.fd(), 0, [&sh](int events) {
    sh.processEvents(events);
    return 0;
  }));

  int ret = write_byte(sockets_[1]);
  ASSERT_GT(ret, 0);
  poll_set_.update(1);

  ASSERT_EQ(sh.bytes_read_, 0);

  ASSERT_TRUE(poll_set_.addEvents(sh.fd(), POLLIN));
  poll_set_.update(1);
  ASSERT_EQ(sh.bytes_read_, 1);

  ret = write_byte(sockets_[1]);
  ASSERT_GT(ret, 0);
  poll_set_.update(1);
  ASSERT_EQ(sh.bytes_read_, 2);

  ASSERT_TRUE(poll_set_.delEvents(sh.fd(), POLLIN));
  ret = write_byte(sockets_[1]);
  ASSERT_GT(ret, 0);
  poll_set_.update(1);
  ASSERT_EQ(sh.bytes_read_, 2);

  ASSERT_TRUE(poll_set_.addEvents(sh.fd(), POLLIN));
  poll_set_.update(1);
  ASSERT_EQ(sh.bytes_read_, 3);

  ASSERT_TRUE(poll_set_.delSocket(poll_fd(sockets_[0])));
  poll_set_.update(1);
  ASSERT_EQ(sh.bytes_read_, 3);
}

TEST_F(Poller, write)
{
  SocketHelper sh(sockets_[0]);
  ASSERT_TRUE(poll_set_.addSocket(sh.fd(), 0, [&sh](int events) {
    sh.processEvents(events);
    return 0;
  }));
  ASSERT_TRUE(poll_set_.addEvents(sh.fd(), POLLOUT));

  poll_set_.update(1);

  ASSERT_EQ(sh.pollouts_received_, 1);
  ASSERT_EQ(sh.bytes_written_, 1);

  ASSERT_TRUE(poll_set_.delEvents(sh.fd(), POLLOUT));
  poll_set_.update(1);
  ASSERT_EQ(sh.pollouts_received_, 1);
  ASSERT_EQ(sh.bytes_written_, 1);
}

TEST_F(Poller, readAndWrite)
{
  SocketHelper sh1(sockets_[0]);
  SocketHelper sh2(sockets_[1]);
  ASSERT_TRUE(poll_set_.addSocket(sh1.fd(), 0, [&sh1](int events) {
    sh1.processEvents(events);
    return 0;
  }));

  ASSERT_TRUE(poll_set_.addSocket(sh2.fd(), 0, [&sh2](int events) {
    sh2.processEvents(events);
    return 0;
  }));

  ASSERT_TRUE(poll_set_.addEvents(sh1.fd(), POLLIN));
  ASSERT_TRUE(poll_set_.addEvents(sh2.fd(), POLLIN));

  sh1.write();
  sh2.write();

  ASSERT_EQ(sh1.bytes_written_, 1);
  ASSERT_EQ(sh2.bytes_written_, 1);

  poll_set_.update(1);

  ASSERT_EQ(sh1.bytes_read_, 1);
  ASSERT_EQ(sh2.bytes_read_, 1);

  ASSERT_TRUE(poll_set_.addEvents(sh1.fd(), POLLOUT));
  ASSERT_TRUE(poll_set_.addEvents(sh2.fd(), POLLOUT));

  poll_set_.update(1);

  ASSERT_EQ(sh1.bytes_written_, 2);
  ASSERT_EQ(sh2.bytes_written_, 2);

  ASSERT_TRUE(poll_set_.delEvents(sh1.fd(), POLLOUT));
  ASSERT_TRUE(poll_set_.delEvents(sh2.fd(), POLLOUT));

  poll_set_.update(1);

  ASSERT_EQ(sh1.bytes_read_, 2);
  ASSERT_EQ(sh2.bytes_read_, 2);
}

TEST_F(Poller, multiAddDel)
{
  SocketHelper sh(sockets_[0]);
  ASSERT_TRUE(poll_set_.addSocket(sh.fd(), 0, [&sh](int events) {
    sh.processEvents(events);
    return 0;
  }));
  // Should return false if adding same socket twice.
  ASSERT_FALSE(poll_set_.addSocket(sh.fd(), 0, [&sh](int events) {
    sh.processEvents(events);
    return 0;
  }));

  ASSERT_TRUE(poll_set_.addEvents(sh.fd(), 0));
  // Expecting false because sh.fd()+1 is not added to PollSet.
  ASSERT_FALSE(poll_set_.addEvents(sh.fd() + 1, 0));

  ASSERT_TRUE(poll_set_.delEvents(sh.fd(), 0));
  // Expecting false because sh.fd()+1 is not added to PollSet.
  ASSERT_FALSE(poll_set_.delEvents(sh.fd() + 1, 0));

  // Expecting false because sh.fd()+1 is not added to PollSet.
  ASSERT_FALSE(poll_set_.delSocket(sh.fd() + 1));
  ASSERT_TRUE(poll_set_.delSocket(sh.fd()));
}

void addThread(PollSet* ps, SocketHelper* sh, Barrier* barrier)
{
  barrier->wait();

  ps->addSocket(sh->fd(), 0, [sh](int events) {
    sh->processEvents(events);
    return 0;
  });
  ps->addEvents(sh->fd(), POLLIN);
  ps->addEvents(sh->fd(), POLLOUT);
}

void delThread(PollSet* ps, SocketHelper* sh, Barrier* barrier)
{
  barrier->wait();

  ps->delEvents(sh->fd(), POLLIN);
  ps->delEvents(sh->fd(), POLLOUT);
  ps->delSocket(sh->fd());
}

/**
 * This test has been disabled. The underlying logic which it tests has three
 * different implementations (poll, epoll, Windows), and development of the epoll
 * version exposed that the test was validating a buggy aspect of the original
 * poll version. To reenable this test, the poll version and the test would both
 * have to be updated.
 *
 * For more discussion, see: https://github.com/ros/ros_comm/pull/1217
 */
TEST_F(Poller, DISABLED_addDelMultiThread)
{
  for (int i = 0; i < 100; ++i)
  {
    SocketHelper sh1(sockets_[0]);
    SocketHelper sh2(sockets_[1]);

    const int thread_count = 100;

    {
      Barrier barrier(thread_count + 1);
      std::vector<std::thread> tg;
      for (int j = 0; j < thread_count/2; ++j)
      {
        tg.emplace_back(addThread, &poll_set_, &sh1, &barrier);
        tg.emplace_back(addThread, &poll_set_, &sh2, &barrier);
      }

      barrier.wait();

      for (auto& t: tg)
          t.join();

      poll_set_.update(1);

      ASSERT_TRUE(sh1.bytes_read_ == 0 || sh1.bytes_read_ == 1);
      ASSERT_TRUE(sh2.bytes_read_ == 0 || sh2.bytes_read_ == 1);
      ASSERT_EQ(sh1.bytes_written_, 1);
      ASSERT_EQ(sh2.bytes_written_, 1);

      poll_set_.update(1);

      ASSERT_TRUE(sh1.bytes_read_ == 1 || sh1.bytes_read_ == 2);
      ASSERT_TRUE(sh2.bytes_read_ == 1 || sh2.bytes_read_ == 2);
      ASSERT_EQ(sh1.bytes_written_, 2);
      ASSERT_EQ(sh2.bytes_written_, 2);
    }

    {
      Barrier barrier(thread_count + 1);
      std::vector<std::thread> tg;
      for (int j = 0; j < thread_count/2; ++j)
      {
        tg.emplace_back(delThread, &poll_set_, &sh1, &barrier);
        tg.emplace_back(delThread, &poll_set_, &sh2, &barrier);
      }

      barrier.wait();

      for (auto& t: tg)
        t.join();

      poll_set_.update(1);

      ASSERT_TRUE(sh1.bytes_read_ == 1 || sh1.bytes_read_ == 2);
      ASSERT_TRUE(sh2.bytes_read_ == 1 || sh2.bytes_read_ == 2);
      ASSERT_EQ(sh1.bytes_written_, 2);
      ASSERT_EQ(sh2.bytes_written_, 2);
    }
  }
}

void addDelManyTimesThread(PollSet* ps, SocketHelper* sh1, SocketHelper* sh2, Barrier* barrier, int count, std::atomic<bool>* done)
{
  done->store(false);

  barrier->wait();

  for (int i = 0; i < count; ++i)
  {
    ps->addSocket(sh1->fd(), 0, [sh1](int events) {
      sh1->processEvents(events);
      return 0;
    });
    ps->addEvents(sh1->fd(), POLLIN);
    ps->addEvents(sh1->fd(), POLLOUT);

    ps->addSocket(sh2->fd(), 0, [sh2](int events) {
      sh2->processEvents(events);
      return 0;
    });
    ps->addEvents(sh2->fd(), POLLIN);
    ps->addEvents(sh2->fd(), POLLOUT);

    std::this_thread::sleep_for(std::chrono::microseconds(100));

    ps->delEvents(sh1->fd(), POLLIN);
    ps->delEvents(sh1->fd(), POLLOUT);
    ps->delSocket(sh1->fd());

    ps->delEvents(sh2->fd(), POLLIN);
    ps->delEvents(sh2->fd(), POLLOUT);
    ps->delSocket(sh2->fd());
  }

  done->store(true);
}

TEST_F(Poller, updateWhileAddDel)
{
  SocketHelper sh1(sockets_[0]);
  SocketHelper sh2(sockets_[1]);

  Barrier barrier(2);
  std::atomic<bool> done{false};
  const int count = 1000;

  std::thread t(addDelManyTimesThread, &poll_set_, &sh1, &sh2, &barrier, count, &done);

  barrier.wait();

  while (!done.load())
  {
    poll_set_.update(1);
  }

  ASSERT_TRUE(sh1.bytes_read_ > 0);
  ASSERT_TRUE(sh1.bytes_written_ > 0);
  ASSERT_TRUE(sh2.bytes_read_ > 0);
  ASSERT_TRUE(sh2.bytes_written_ > 0);
  t.join();
}

TEST_F(Poller, signal)
{
  // first one clears out any calls to signal() caused by construction
  poll_set_.update(0);

  std::thread t(&Poller::waitThenSignal, this);
  poll_set_.update(-1);

  // wait for poll_set_.signal_mutex_ to be unlocked after invoking signal()
  std::this_thread::sleep_for(std::chrono::microseconds(50000));
  t.join();
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  miniros::handleCrashes();
  miniros::ensureNetworkInitialized();

#ifndef _WIN32
  signal(SIGPIPE, SIG_IGN);
#endif

  return RUN_ALL_TESTS();
}

