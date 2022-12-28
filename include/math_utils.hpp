#ifndef MATH_UTILS_HPP
#define MATH_UTILS_HPP

#include <algorithm>
#include <cmath>
#include <numbers>

namespace {
namespace LPCC {

auto atan2_degree(const auto y, const auto x) {
    // return [-180.f, 180.f]
    return std::atan2(y, x) * 180.f / std::numbers::pi_v<float>;
}

auto degree_to_radian(const float degree) {
    return degree * std::numbers::pi_v<float> / 180.f;
}

} // namespace LPCC
} // anonymous namespace

#endif // MATH_UTILS_HPP
