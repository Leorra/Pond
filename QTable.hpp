/*] ============================================== [*]
[*] A very serious the Frozen Pond problem project [*]
[*] C++ 23 Code Standard, SoA SIMD friendly Design [*]
[*]     (^_^) https://github.com/Leorra (^_^)      [*]
[*] ============================================== [*/

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "Pond.hpp"
#include "XorShift32.hpp"

namespace pond {

	// Q-Table implementation for reinforcement learning, parameterized by a grid type (e.g., Pond)
	template <typename Grid>
	class alignas(64) QTable {
	private:
		// Hyperparameters
		static constexpr float kDefaultEpsilon_ = 0.5f; // Initial exploration rate
		static constexpr double gamma_ = 0.99;          // Discount factor
		static constexpr double step_ = -0.01;          // Step reward penalty

		static constexpr double kGoalReward_ = 100.0;   // Terminal reward: goal
		static constexpr double kHoleReward_ = -5.0;    // Terminal reward: hole

		static constexpr std::size_t width_ = Grid::getWidth();
		static constexpr std::size_t height_ = Grid::getHeight();

		static constexpr std::size_t kNumActions_ = static_cast<std::size_t>(Direction::Count);
		static constexpr std::uint32_t kDefaultSeed_ = 1337U;
		static constexpr double kInfinity_ = std::numeric_limits<double>::infinity();

		// SoA implementation of the Q-table for better cache locality and performance
		std::array<std::array<double, width_* height_>, kNumActions_> q_table_ {};
		std::array<std::array<std::size_t, width_* height_>, kNumActions_> q_visits_ {};
		const Grid& pond_; // Reference to the grid (Pond) for which the Q-table is being maintained

		float epsilon_ { kDefaultEpsilon_ }; // Exploration rate, adjustable at runtime
		mutable XorShift32 rng_ { kDefaultSeed_ };

		// Mean Q-value for a position and direction: -infinity if the move leaves
		// the grid, 0.0 if unvisited.
		[[nodiscard]] double getQValue(Position pos, Direction dir) const noexcept {
			const auto idx = static_cast<std::size_t>(dir);
			if (idx >= kNumActions_) [[unlikely]] { return -kInfinity_; }
			if (!Grid::getNextPosition(pos, dir).has_value()) [[unlikely]] { return -kInfinity_; }
			const auto pos_idx = Grid::getIndex(pos);
			if (!pos_idx.has_value()) [[unlikely]] { return -kInfinity_; }
			const std::size_t visits = q_visits_[idx][*pos_idx];
			if (visits == 0) [[unlikely]] { return 0.0; }
			return q_table_[idx][*pos_idx] / static_cast<double>(visits);
		}

		// Uniformly random in-bounds direction, for e-greedy exploration
		[[nodiscard]] Direction getRandomValidDirection(Position pos) const noexcept {
			std::array<Direction, kNumActions_> valid_dirs {};
			std::size_t valid_count = 0;
			for (std::size_t i = 0; i < kNumActions_; ++i) {
				const auto dir = static_cast<Direction>(i);
				if (Grid::getNextPosition(pos, dir).has_value()) { valid_dirs[valid_count++] = dir; }
			}
			if (valid_count == 0) [[unlikely]] { return Direction::Count; }
			const std::size_t random_index = rng_.getRandomInt(static_cast<std::uint32_t>(valid_count));
			return valid_dirs[random_index];
		}

		// Max Q-value action, tie-broken randomly. Off-grid actions are excluded.
		[[nodiscard]] Direction getMaxQDirection(Position pos) const noexcept {
			static constexpr double kEpsilon = 1e-6;
			double max_q = -kInfinity_;
			std::array<Direction, kNumActions_> best_actions {};
			std::size_t count = 0;

			for (std::size_t i = 0; i < kNumActions_; ++i) {
				const auto dir = static_cast<Direction>(i);
				const double q_value = getQValue(pos, dir);
				if (q_value == -kInfinity_) [[unlikely]] { continue; }
				if (q_value > max_q + kEpsilon) {
					max_q = q_value;
					best_actions[0] = dir;
					count = 1;
				} else if (q_value >= max_q - kEpsilon) {
					best_actions[count++] = dir;
				}
			}
			if (count == 0) [[unlikely]] { return Direction::Count; }
			const std::size_t index = rng_.getRandomInt(static_cast<std::uint32_t>(count));
			return best_actions[index];
		}

		// Make a move according to the e-greedy strategy
		[[nodiscard]] Direction makeMove(Position pos) const noexcept {
			if (rng_.getRandomFloat() < epsilon_) { return getRandomValidDirection(pos); }
			return getMaxQDirection(pos);
		}

	public:
		explicit QTable(const Grid& pond, std::uint32_t seed = kDefaultSeed_)
			: pond_(pond), rng_(seed) {
		}

		// Sets the exploration rate. Returns false (value unchanged) if it is outside [0, 1] or NaN.
		[[nodiscard]] bool setEpsilon(const float epsilon) noexcept {
			if (!(epsilon >= 0.0f && epsilon <= 1.0f)) [[unlikely]] { return false; }
			epsilon_ = epsilon;
			return true;
		}

		[[nodiscard]] float getEpsilon() const noexcept { return epsilon_; }

		void setSeed(const std::uint32_t seed) noexcept { rng_.setSeed(seed); }

		// Generates one e-greedy episode into 'path'. Returns true iff a terminal
		// cell (goal or hole) was reached; false denotes a truncated or failed episode.
		[[nodiscard]] bool createSinglePath(auto& path) noexcept {
			path.clear();

			Position current = pond_.getStart();
			if (!path.push(current)) [[unlikely]] { return false; }
			if (pond_.isTerminal(current)) [[unlikely]] { return true; }

			// The start position occupies one slot; push() has succeeded, hence
			// capacity() >= 1 and the subtraction cannot underflow.
			const std::size_t max_steps = path.capacity() - 1;
			for (std::size_t n = 0; n < max_steps; ++n) {
				const Direction dir = makeMove(current);
				if (dir == Direction::Count) [[unlikely]] { return false; }
				const auto next_opt = Grid::getNextPosition(current, dir);
				if (!next_opt.has_value()) [[unlikely]] { return false; }
				current = *next_opt;
				if (!path.push(current)) [[unlikely]] { return false; }
				if (pond_.isTerminal(current)) { return true; }
			} return false;
		}

		// Every-visit Monte Carlo update: accumulates the discounted return of each
		// (position, action) pair into the sum and visit tables. Also accepts truncated
		// episodes: their last transition then carries only the step penalty (no terminal reward).
		// Returns false if the path is too short or contains a non-adjacent transition.
		[[nodiscard]] bool updateTable(const auto& path) noexcept {
			if (path.size() < 2) { return false; }

			auto it = path.rbegin();
			const auto rend = path.rend();

			Position to = *it; // Last position of the episode
			const Type last_type = pond_.getType(to);
			double reward = step_;
			if (last_type == Type::Goal) { reward += kGoalReward_; } else if (last_type == Type::Hole) { reward += kHoleReward_; }

			double ret = 0.0;
			for (++it; it != rend; ++it) {
				const Position from = *it;
				const Direction dir = Grid::getDirection(from, to);
				const auto from_idx = Grid::getIndex(from);
				if (dir == Direction::Count || !from_idx.has_value()) [[unlikely]] { return false; }
				ret = reward + gamma_ * ret;
				const auto a = static_cast<std::size_t>(dir);
				q_table_[a][*from_idx] += ret;
				++q_visits_[a][*from_idx];
				reward = step_; // Only the final transition carries the terminal reward
				to = from;
			} return true;
		}
	};

} // namespace pond