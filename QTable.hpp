#pragma once

#include <array>
#include <cstdint>
#include <limits>

#include "Pond.hpp"
#include "XorShift32.hpp"

namespace pond {

	// Q-Table implementation for reinforcement learning, parameterized by a grid type (e.g., Pond)
	template <typename Grid>
	class alignas(64) QTable {
	private:
		static constexpr std::size_t width_ = Grid::getWidth();
		static constexpr std::size_t height_ = Grid::getHeight();

		static constexpr std::size_t kNumActions_ = static_cast<std::size_t>(Direction::Count);
		static constexpr std::uint32_t kDefaultSeed_ = 1337U;
		static constexpr float kInfinity_ = static_cast<float>(std::numeric_limits<float>::infinity());


		// SoA implementation of the Q-table for better cache locality and performance
		std::array<std::array<float, width_* height_>, kNumActions_> q_table_ {};
		std::array<std::array<std::size_t, width_* height_>, kNumActions_> q_visits_ {};
		const Grid& pond_; // Reference to the grid (Pond) for which the Q-table is being maintained

		mutable XorShift32 rng_ { kDefaultSeed_ };

		// Get a random direction from the set of possible actions for e-greedy exploration
		[[nodiscard]] Direction getRandomValidDirection(Position pos) noexcept {
			std::array<Direction, kNumActions_> valid_dirs {};
			std::size_t valid_count = 0;
			for (std::size_t i = 0; i < kNumActions_; ++i) {
				const auto dir = static_cast<Direction>(i);
				if (Grid::getNextPosition(pos, dir).has_value()) { valid_dirs[valid_count++] = dir; }
			} if (valid_count == 0) [[unlikely]] { return Direction::Count; }
			const std::size_t random_index = rng_.getRandomInt(static_cast<std::uint32_t>(valid_count));
			return valid_dirs[random_index];
		}

		// Get the Q-value for a given position and direction,
		// returning -infinity if out of bounds or 0.0f for unvisited
		[[nodiscard]] float getQValue(Position pos, Direction dir) const noexcept {
			const auto idx = static_cast<std::size_t>(dir);
			if (idx >= kNumActions_) [[unlikely]] { return -kInfinity_; }
			if (!Grid::getNextPosition(pos, dir).has_value()) [[unlikely]] { return -kInfinity_; }
			const auto pos_idx = Grid::getIndex(pos);
			if (!pos_idx.has_value()) [[unlikely]] { return -kInfinity_; }
			const std::size_t visits = q_visits_[idx][*pos_idx];
			if (visits == 0) [[unlikely]] { return 0.0f; }
			const float q_value = q_table_[idx][*pos_idx];
			return q_value / static_cast<float>(visits);
		}

		// Get Max Q-value action, randomly chosen among tied float values.
		// Actions leading off the grid (getQValue == -infinity) are excluded outright.
		[[nodiscard]] Direction getMaxQAction(Position pos) const noexcept {
			static constexpr float kEpsilon = 1e-6f;
			float max_q = -kInfinity_;
			std::array<Direction, kNumActions_> best_actions {};
			std::size_t count = 0;
			for (std::size_t i = 0; i < kNumActions_; ++i) {
				const auto dir = static_cast<Direction>(i);
				const float q_value = getQValue(pos, dir);
				if (q_value == -kInfinity_) [[unlikely]] { continue; }
				const float diff = q_value - max_q;
				if (diff > kEpsilon) {
					max_q = q_value; best_actions[0] = dir; count = 1;
				} else if (diff >= -kEpsilon) { best_actions[count++] = dir; }
			} if (count == 0) [[unlikely]] { return Direction::Count; }
			const std::size_t index = rng_.getRandomInt(static_cast<std::uint32_t>(count));
			return best_actions[index];
		}

	public:
		explicit QTable(const Grid& pond, std::uint32_t seed = kDefaultSeed_)
			: pond_(pond), rng_(seed) {
		}
	};

} // namespace pond