/*
 * @file Timer.h
 * @brief countting time tools for debug
 * @author Jing Dong
 * @date May 11, 2014
 */

#pragma once

#include <iostream>

#ifndef _WIN32
// UNIX interface
#include <sys/time.h>

#else
// Windows replacement
// See http://web.archive.org/web/20130406033313/http://suacommunity.com/dictionary/gettimeofday-entry.php
#include <time.h>
#include <windows.h>

#if defined(_MSC_VER) || defined(_MSC_EXTENSIONS)
#define DELTA_EPOCH_IN_MICROSECS  11644473600000000Ui64
#else
#define DELTA_EPOCH_IN_MICROSECS  11644473600000000ULL
#endif

struct timezone {
  int  tz_minuteswest; /* minutes W of Greenwich */
  int  tz_dsttime;     /* type of dst correction */
};

// Definition of a gettimeofday function

int gettimeofday(struct timeval *tv, struct timezone *tz) {
  // Define a structure to receive the current Windows filetime
  FILETIME ft;

  // Initialize the present time to 0 and the timezone to UTC
  unsigned __int64 tmpres = 0;
  static int tzflag = 0;

  if (NULL != tv) {
    GetSystemTimeAsFileTime(&ft);

    // The GetSystemTimeAsFileTime returns the number of 100 nanosecond 
    // intervals since Jan 1, 1601 in a structure. Copy the high bits to 
    // the 64 bit tmpres, shift it left by 32 then or in the low 32 bits.
    tmpres |= ft.dwHighDateTime;
    tmpres <<= 32;
    tmpres |= ft.dwLowDateTime;

    // Convert to microseconds by dividing by 10
    tmpres /= 10;

    // The Unix epoch starts on Jan 1 1970.  Need to subtract the difference 
    // in seconds from Jan 1 1601.
    tmpres -= DELTA_EPOCH_IN_MICROSECS;

    // Finally change microseconds to seconds and place in the seconds value. 
    // The modulus picks up the microseconds.
    tv->tv_sec = (long)(tmpres / 1000000UL);
    tv->tv_usec = (long)(tmpres % 1000000UL);
  }

  if (NULL != tz) {
    if (!tzflag) {
      _tzset();
      tzflag++;
    }
    // Adjust for the timezone west of Greenwich
    tz->tz_minuteswest = _timezone / 60;
    tz->tz_dsttime = _daylight;
  }

  return 0;
}
#endif


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
