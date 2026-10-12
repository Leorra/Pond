/*
======================================================
[+] A very serious the Frozen Pond problem project [+]
[+] C++ 23 Code Standard, SoA SIMD friendly Design [+]
[+] https://github.com/Leorra                      [+]
======================================================
*/

#pragma once

#include <array>
#include <cstdint>
#include <optional>

namespace pond {

	enum class Type : std::uint8_t { Empty, Start, Goal, Hole, Path, Count }; // Node types in the pond
	enum class Direction : std::uint8_t { Up, Down, Left, Right, Count }; // Possible movement directions

	struct Position {
		std::size_t x_; std::size_t y_;

		[[nodiscard]] constexpr bool operator==(const Position&) const noexcept = default;
	};

	// Basic Pond logic and representation, with a grid of cells of according types.
	template <std::size_t width_, std::size_t height_>
	class alignas(64) Pond {
	private:
		std::array<Type, width_* height_> grid_ {};

		Position start_ { 0, 0 }; // Start position (top-left corner)
		Position goal_ { width_ - 1, height_ - 1 }; // Goal position (bottom-right corner)

	public:
		Pond() { grid_[0] = Type::Start; grid_[width_ * height_ - 1] = Type::Goal; }

		// Get access to the grid_ array
		[[nodiscard]] const std::array<Type, width_* height_>& getGrid() const noexcept { return grid_; }

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

		[[nodiscard]] bool isTerminal(Position pos) const noexcept {
			const Type type = getType(pos);
			if (type == Type::Hole || type == Type::Goal) { return true; }
			return false;
		}

		// Get the type of a cell at a given position, returning Type::Count if out of bounds
		[[nodiscard]] Type getType(Position pos) const noexcept {
			const auto idx = getIndex(pos);
			if (!idx.has_value()) [[unlikely]] { return Type::Count; }
			return grid_[*idx];
		}

		// Set the type of a cell at a given position, returning false if out of bounds
		[[nodiscard]] bool setType(Position pos, Type type) noexcept {
			const auto idx = getIndex(pos);
			if (!idx.has_value()) [[unlikely]] { return false; }
			if (type == Type::Start) {
				const auto idx_start = getIndex(start_);
				grid_[*idx_start] = Type::Empty; // Clear previous start position]
				start_ = pos;
			}
			if (type == Type::Goal) {
				const auto idx_goal = getIndex(goal_);
				grid_[*idx_goal] = Type::Empty; // Clear previous goal position
				goal_ = pos;
			}
			grid_[*idx] = type;
			return true;
		}

		// Get the start position
		[[nodiscard]] Position getStart() const noexcept { return start_; }

		// Get the goal position
		[[nodiscard]] Position getGoal() const noexcept { return goal_;	}

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

} // namespace pond