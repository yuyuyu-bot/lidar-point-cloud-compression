#ifndef TIMER_HPP
#define TIMER_HPP

#include <chrono>
#include <iostream>
#include <stack>

namespace LPCC {

class Timer {
private:
    auto now() {
        return std::chrono::system_clock::now();
    }

public:
    using ClockType = std::chrono::system_clock::time_point;
    using TimeType = std::chrono::microseconds;

    void tick() {
        stack.push(now());
    }

    auto tock() {
        const auto f = now();
        const auto s = stack.top();
        stack.pop();
        return std::chrono::duration_cast<TimeType>(f - s).count();
    }

private:
    std::stack<ClockType> stack;
};

} // namespace LPCC

#endif // TIMER_HPP
