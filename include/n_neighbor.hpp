#ifndef N_NEIGHBOR_HPP
#define N_NEIGHBOR_HPP

#include <cstddef>

namespace {
namespace LPCC {

class NNeighbor {
public:
    template <int N>
    static constexpr auto len() {
        if constexpr (N == 4) {
            return LenN4;
        }
        else if constexpr (N == 8) {
            return LenN8;
        }
        else {
            return len<4>();
        }
    }

    template <int N>
    static constexpr auto X() {
        if constexpr (N == 4) {
            return N4x;
        }
        else if constexpr (N == 8) {
            return N8x;
        }
        else {
            return X<4>();
        }
    }

    template <int N>
    static constexpr auto Y() {
        if constexpr (N == 4) {
            return N4y;
        }
        else if constexpr (N == 8) {
            return N8y;
        }
        else {
            return Y<4>();
        }
    }

private:
    static constexpr std::size_t LenN4 = 4;
    static constexpr int N4x[4] = {+1, +0, -1, +0};
    static constexpr int N4y[4] = {+0, +1, +0, -1};
    static constexpr std::size_t LenN8 = 8;
    static constexpr int N8x[8] = {+1, +1, +0, -1, -1, -1, +0, +1};
    static constexpr int N8y[8] = {+0, +1, +1, +1, +0, -1, -1, -1};
};

} // namespace LPCC
} // anonymous namespace

#endif // N_NEIGHBOR_HPP
