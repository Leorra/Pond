#pragma once

#include <cstddef>

namespace pond {

	// Basic stack class for storing sequences of steps, with a fixed capacity
	template <typename T, std::size_t Capacity>
	struct Path {
		std::array<T, Capacity> data_ {};
		std::size_t count_ = 0;

		inline bool push(const T& item) noexcept {
			if (count_ < Capacity) [[likely]] { data_[count_++] = item; return true; } return false;
		}

		inline void clear() noexcept { count_ = 0; }
		[[nodiscard]] inline std::size_t size() const noexcept { return count_; }
		[[nodiscard]] inline bool empty() const noexcept { return count_ == 0; }

		[[nodiscard]] inline auto rbegin() const noexcept { return std::make_reverse_iterator(data_.begin() + count_); }
		[[nodiscard]] inline auto rend() const noexcept { return std::make_reverse_iterator(data_.begin()); }
	};

} // nameaspace pond