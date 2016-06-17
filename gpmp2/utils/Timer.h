/*
 * @file Timer.h
 * @brief countting time tools for debug
 * @author Jing Dong
 * @date May 11, 2014
 */

#pragma once

#include <sys/time.h>
#include <iostream>

namespace gpmp2 {

/* ************************************************************************* */
// timer class
class Timer {

private:
  std::string str_;
  timeval starttime_, endtime_;
  signed long timer_usec_;   // accumulator

public:
  Timer() : str_(), timer_usec_(0) {}
  Timer(std::string str) : str_(str), timer_usec_(0) {}
  virtual ~Timer() {}

  // ===================================================================
  // one-time timer
  // start timer
  void tic() {
    gettimeofday(&starttime_, 0);
  }

  // stop timer and get time
  signed long toc() {
    gettimeofday(&endtime_, 0);
    signed long usec = 1000000 * (static_cast<signed long>(endtime_.tv_sec)
        - static_cast<signed long>(starttime_.tv_sec))
        + static_cast<signed long>(endtime_.tv_usec)
        - static_cast<signed long>(starttime_.tv_usec);
    std::cout << str_ << " time used: " << usec << "us" << std::endl;
    return usec;
  }

  // ===================================================================
  // accumulated timer
  // start timer
  void start() {
    gettimeofday(&starttime_, 0);
  }
  // stop timer
  void stop() {
    gettimeofday(&endtime_, 0);
    timer_usec_ += 1000000 * (static_cast<signed long>(endtime_.tv_sec)
        - static_cast<signed long>(starttime_.tv_sec))
        + static_cast<signed long>(endtime_.tv_usec)
        - static_cast<signed long>(starttime_.tv_usec);
  }
  // dispaly time
  signed long showtime() {
    std::cout << str_ << " time used: " << timer_usec_ << "us" << std::endl;
    return timer_usec_;
  }
  // clear internal
  void clear() {
    timer_usec_ = 0;
  }
};

}   // namespace gpmp2
