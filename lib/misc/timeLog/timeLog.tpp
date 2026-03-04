#pragma once

#include <chrono>
#include <ctime>
#include <iomanip>

inline TimeLogStream::TimeLogStream(bool dodt) {
    using namespace std::chrono;

    auto now = system_clock::now();
    auto us = duration_cast<microseconds>(
        now.time_since_epoch());


    std::time_t t = system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&t);

    std::cout << '['
              << std::put_time(&tm, "%H:%M:%S")
              << '.' << std::setw(6) << std::setfill('0') << (us % seconds(1)).count()
              << "] ";
    if (dodt){ //dt stuff, very simple and hacky
        std::cout << "(+" << std::setw(6) << std::setfill('0') << (us-microseconds(__timelogger_last_us)).count() << ") ";
        __timelogger_last_us = us.count();
    }
}
 
template<typename T>
inline TimeLogStream& TimeLogStream::operator<<(const T& value) {
    std::cout << value;
    return *this;
}

inline TimeLogStream&
TimeLogStream::operator<<(std::ostream& (*manip)(std::ostream&)) {
    std::cout << manip;
    return *this;
}
