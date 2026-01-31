#pragma once

#include <chrono>
#include <ctime>
#include <iomanip>

inline TimeLogStream::TimeLogStream() {
    using namespace std::chrono;

    auto now = system_clock::now();
    auto us = duration_cast<microseconds>(
        now.time_since_epoch()) % seconds(1);

    std::time_t t = system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&t);

    std::cout << '['
              << std::put_time(&tm, "%H:%M:%S")
              << '.' << std::setw(6) << std::setfill('0') << us.count()
              << "] ";
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
