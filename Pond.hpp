#pragma once

#include <array>
#include <cstdint>
#include <cstdlib>
#include <iterator>
#include <limits>
#include <optional>

#include "XorShift32.hpp"

namespace pond {

	enum class Type : std::uint8_t { Empty, Start, Goal, Hole, Path, Count }; // Node types in the pond
	enum class Direction : std::uint8_t { Up, Down, Left, Right, Count }; // Possible movement directions

	struct Position {
		std::size_t x_;
		std::size_t y_;
		[[nodiscard]] constexpr bool operator==(const Position&) const noexcept = default;
	};

	// Basic Pond logic and representation, with a grid of cells of according types.
	template <std::size_t width_, std::size_t height_>
	class alignas(64) Pond {
	private:
		std::array<Type, width_* height_> grid_ {};

	public:
		Pond() { grid_[0] = Type::Start; grid_[width_ * height_ - 1] = Type::Goal; }

		// Get the dimensions of the pond
		[[nodiscard]] static constexpr std::size_t getWidth() noexcept { return width_; }
		[[nodiscard]] static constexpr std::size_t getHeight() noexcept { return height_; }

		// Check if a position is within the bounds of the pond
		[[nodiscard]] static constexpr bool inBounds(Position pos) noexcept {
			return pos.x_ < width_ && pos.y_ < height_;
		}

		// Get the index of a position in the grid array, or std::nullopt if out of bounds
		[[nodiscard]] static constexpr std::optional<std::size_t> getIndex(Position pos) noexcept {
			if (!inBounds(pos)) { return std::nullopt; }
			return pos.y_ * width_ + pos.x_;
		}

		// Get the type of a cell at a given position, returning Type::Count if out of bounds
		[[nodiscard]] Type getType(Position pos) const noexcept {
			const auto idx = getIndex(pos);
			if (!idx.has_value()) [[unlikely]] { return Type::Count; }
			return grid_[*idx];
		}

		// Set the type of a cell at a given position, returning the old type or Type::Count if out of bounds/invalid.
		// Guarantees uniqueness for Start and Goal types by clearing previous instances to Type::Empty.
		// Start and Goal can only overwrite cells that are currently Empty (or already Start/Goal).
		[[nodiscard]] Type setType(Position pos, Type type) noexcept {
			const auto idx = getIndex(pos);
			if (!idx.has_value()) [[unlikely]] { return Type::Count; }
			const Type target_cell = grid_[*idx];
			if (type == Type::Start || type == Type::Goal) {
				if (target_cell != Type::Empty && target_cell != type) { return Type::Count; }
				for (auto& cell : grid_) {
					if (cell == type) { cell = Type::Empty; }
				}
			}
			const Type old_type = target_cell;
			grid_[*idx] = type;
			return old_type;
		}

		// Get the position of the start or goal cell, if it exists and is unique
		// Returns std::nullopt if the type is not Start or Goal, or if there are multiple cells of that type
		[[nodiscard]] std::optional<Position> getStartGoal(const Type type) const noexcept {
			if (type != Type::Start && type != Type::Goal) [[unlikely]] { return std::nullopt; }
			std::optional<Position> found;
			for (std::size_t i = 0; i < grid_.size(); ++i) {
				if (grid_[i] == type) {
					if (found.has_value()) [[unlikely]] { return std::nullopt; }
					found = Position { i % width_, i / width_ };
				}
			} return found;
		}

		// Get the next position in the given direction, or std::nullopt if out of bounds
		[[nodiscard]] static constexpr std::optional<Position> getNextPosition(Position pos, Direction dir) noexcept {
			const auto idx = static_cast<std::size_t>(dir);
			if (idx >= static_cast<std::size_t>(Direction::Count)) [[unlikely]] { return std::nullopt; }
			static constexpr std::array<std::ptrdiff_t, 4> kDx { 0, 0, -1, 1 };
			static constexpr std::array<std::ptrdiff_t, 4> kDy { -1, 1, 0, 0 };
			const auto nx = static_cast<std::ptrdiff_t>(pos.x_) + kDx[idx];
			const auto ny = static_cast<std::ptrdiff_t>(pos.y_) + kDy[idx];
			if (nx < 0 || nx >= static_cast<std::ptrdiff_t>(width_) ||
				ny < 0 || ny >= static_cast<std::ptrdiff_t>(height_)) {
				return std::nullopt;
			} return Position { static_cast<std::size_t>(nx), static_cast<std::size_t>(ny) };
		}

		// Utility method for Gamma back-tracing
		[[nodiscard]] static constexpr Direction getReversedDirection(const Direction dir) noexcept {
			switch (dir) {
				case Direction::Up: return Direction::Down;
				case Direction::Down: return Direction::Up;
				case Direction::Left:	return Direction::Right;
				case Direction::Right: return Direction::Left;
				default: return Direction::Count;
			}
		}
	};

	// Q-Table implementation for reinforcement learning, parameterized by a grid type (e.g., Pond)
	template <typename Grid>
	class alignas(64) QTable {
	private:
		const Grid& pond_; // Reference to the grid (Pond) for which the Q-table is being maintained

		static constexpr std::size_t width_ = Grid::getWidth();
		static constexpr std::size_t height_ = Grid::getHeight();

		static constexpr std::size_t kNumActions_ = static_cast<std::size_t>(Direction::Count);
		static constexpr std::uint32_t kDefaultSeed_ = 1337U;

		// SoA implementation of the Q-table for better cache locality and performance
		std::array<std::array<float, width_* height_>, kNumActions_> q_table_ {};
		std::array<std::array<std::size_t, width_* height_>, kNumActions_> q_visits_ {};

		mutable XorShift32 rng_ { kDefaultSeed_ };

		// Get the Q-value for a given position and direction,
		// returning -infinity if out of bounds or 0.0f for unvisited
		[[nodiscard]] float getQValue(Position pos, Direction dir) const noexcept {
			const auto idx = static_cast<std::size_t>(dir);
			if (idx >= kNumActions_) [[unlikely]] { return -std::numeric_limits<float>::infinity(); }
			const auto pos_idx = Grid::getIndex(pos);
			if (!pos_idx.has_value()) [[unlikely]] { return -std::numeric_limits<float>::infinity(); }
			const std::size_t visits = q_visits_[idx][*pos_idx];
			if (visits == 0) [[unlikely]] { return 0.0f; }
			const float q_value = q_table_[idx][*pos_idx];
			return q_value / static_cast<float>(visits);
		}

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

		// Get Max Q-value action, randomly chosen among tied float values
		[[nodiscard]] Direction getMaxQAction(Position pos) const noexcept {
			float max_q = -std::numeric_limits<float>::infinity();
			std::array<Direction, kNumActions_> best_actions {};
			std::size_t count = 0;
			static constexpr float kEpsilon = 1e-6f;
			for (std::size_t i = 0; i < kNumActions_; ++i) {
				const auto dir = static_cast<Direction>(i);
				const float q_value = getQValue(pos, dir);
				if (q_value - max_q > kEpsilon) {
					max_q = q_value; best_actions[0] = dir; count = 1;
				} else if (std::abs(q_value - max_q) <= kEpsilon && q_value != -std::numeric_limits<float>::infinity()) {
					best_actions[count++] = dir;
				}
			} if (count == 0) { return Direction::Count; }
			const std::size_t index = rng_.getRandomInt(static_cast<std::uint32_t>(count));
			return best_actions[index];
		}

	public:
		explicit QTable(const Grid& pond, std::uint32_t seed = kDefaultSeed_)
			: pond_(pond), rng_(seed) {
		}
	};

	// Path class to store a sequence of steps, with a fixed capacity
	template <typename T, std::size_t Capacity>
	struct Path {
		std::array<T, Capacity> data {};
		std::size_t count = 0;

		inline bool push(const T& item) noexcept {
			if (count < Capacity) [[likely]] { data[count++] = item; return true; }
			return false;
		}
		inline void clear() noexcept { count = 0; }
		[[nodiscard]] inline std::size_t size() const noexcept { return count; }
		[[nodiscard]] inline bool empty() const noexcept { return count == 0; }

		[[nodiscard]] inline auto rbegin() const noexcept { return std::make_reverse_iterator(data.begin() + count); }
		[[nodiscard]] inline auto rend() const noexcept { return std::make_reverse_iterator(data.begin()); }
	};

} // namespace pond