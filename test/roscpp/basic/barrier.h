/*
 * barrier.h
 *
 *  Created on: Jan 16, 2022
 *      Author: vrobot
 */

#ifndef MINIROSCPP_TEST_ROSCPP_BASIC_BARRIER_H_
#define MINIROSCPP_TEST_ROSCPP_BASIC_BARRIER_H_

#include <mutex>
#include <condition_variable>

/// Replacement for boost::barrier.
/// It should be replaced by std::barrier when c++20 is popular enough. Now we stick to c++14.
class Barrier
{
public:
    explicit Barrier(unsigned int count)
    {
        if (count == 0)
            throw std::runtime_error("Barrier count must be nonzero");
        m_count = count;
        m_initialCount = count;
    }

    Barrier(const Barrier& ) = delete;

    bool wait()
    {
      std::unique_lock < std::mutex > lock(m_mutex);
      unsigned int gen = m_generation;

      if (--m_count == 0)
      {
        m_generation++;
        m_count = m_initialCount;
        assert(m_count != 0);
        m_cond.notify_all();
        return true;
      }

      while (gen == m_generation)
        m_cond.wait(lock);
      return false;
    }

protected:
    std::mutex m_mutex;
    std::condition_variable m_cond;
    unsigned int m_count = 0;
    unsigned int m_initialCount = 0;
    unsigned int m_generation = 0;
};



#endif /* MINIROSCPP_TEST_ROSCPP_BASIC_BARRIER_H_ */
