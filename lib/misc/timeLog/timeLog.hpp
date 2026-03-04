#pragma once

//simple logger that adds time to it. nothing fancy just prints to terminal

#include <iostream>

struct TimeLogStream {
    TimeLogStream(bool dodt = false);

    template<typename T>
    TimeLogStream& operator<<(const T& value);

    TimeLogStream& operator<<(std::ostream& (*manip)(std::ostream&));
};

#define TIMELOG TimeLogStream()
#define TIMELOGDT TimeLogStream(true)

inline std::int64_t __timelogger_last_us = 0;



#include "timeLog.tpp"

