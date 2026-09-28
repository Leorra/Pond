/*
======================================================
[+] A very serious the Frozen Pond problem project [+]
[+] C++ 23 Code Standard, SoA SIMD friendly Design [+]
[+] https://github.com/Leorra                      [+]
======================================================
*/

#pragma once

#include <array>
#include <cstddef>
#include <iterator>

namespace pond {

    // Basic stack class for storing sequences of steps, with a fixed capacity
    template <typename T, std::size_t Capacity>
    class Path {
    private:
        std::array<T, Capacity> data_ {};
        std::size_t count_ = 0;

    public:
        [[nodiscard]] static constexpr std::size_t capacity() noexcept { return Capacity; }

        inline bool push(const T& item) noexcept {
            if (count_ < Capacity) [[likely]] { data_[count_++] = item; return true; }
            return false;
        }

        inline void clear() noexcept { count_ = 0; }
        [[nodiscard]] inline std::size_t size() const noexcept { return count_; }
        [[nodiscard]] inline bool empty() const noexcept { return count_ == 0; }

        [[nodiscard]] inline auto rbegin() const noexcept { return std::make_reverse_iterator(data_.begin() + count_); }
        [[nodiscard]] inline auto rend() const noexcept { return std::make_reverse_iterator(data_.begin()); }
    };

} // namespace pond