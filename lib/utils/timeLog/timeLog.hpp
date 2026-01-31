#pragma once

//simple logger that adds time to it. nothing fancy just prints to terminal

#include <iostream>

struct TimeLogStream {
    TimeLogStream();

    template<typename T>
    TimeLogStream& operator<<(const T& value);

    TimeLogStream& operator<<(std::ostream& (*manip)(std::ostream&));
};

#define TIMELOG TimeLogStream()

#include "timeLog.tpp"
