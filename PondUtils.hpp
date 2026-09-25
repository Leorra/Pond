#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <format>
#include <print>
#include <random>
#include <string_view>

#include "Pond.hpp"
#include "XorShift32.hpp"

namespace pond {

    class Cell {
    private:
        Position pos_ { 0, 0 };
        Type type_ { Type::Empty };

    public:
        explicit Cell(Position pos, Type type) : pos_(pos), type_(type) {}

        [[nodiscard]] Position getPosition() const noexcept { return pos_; }
        [[nodiscard]] Type getType() const noexcept { return type_; }
    };

    template <typename Grid>
    class PondUtils {
    private:
        Grid& pond_;

        static constexpr std::size_t width_ = Grid::getWidth();
        static constexpr std::size_t height_ = Grid::getHeight();

        static constexpr std::array<std::string_view,
            static_cast<std::size_t>(Type::Count)> kTypeStrings { " ", "S", "G", "H", "*" };
        static constexpr std::array<std::string_view, static_cast<std::size_t>(Type::Count)> kTypeColors {
            "\033[97m", "\033[92m", "\033[33m", "\033[94m", "\033[31m"
        };
        static constexpr std::string_view kReset_ = "\033[0m";
        static constexpr std::string_view kCursorOff_ = "\033[?25l";
        static constexpr std::string_view kCursorOn_ = "\033[?25h";

        static constexpr std::size_t kMaxCellBytes_ = [] {
            std::size_t m = 0;
            for (std::size_t i = 0; i < static_cast<std::size_t>(Type::Count); ++i) {
                const std::size_t len = kTypeColors[i].size() + kTypeStrings[i].size() + kReset_.size() + 1;
                if (len > m) m = len;
            } return m;
        }();

        static constexpr std::size_t kBufferSize_ = width_ * height_ * kMaxCellBytes_ + height_ + 64;

        [[nodiscard]] static constexpr std::string_view getTypeString(Type type) noexcept {
            const auto idx = static_cast<std::size_t>(type);
            return idx < kTypeStrings.size() ? kTypeStrings[idx] : kTypeStrings.back();
        }

    public:
        explicit PondUtils(Grid& pond) : pond_(pond) {}

        [[nodiscard]] bool populateHoles(const float rate, std::random_device& rd) noexcept {
            const std::uint32_t seed = rd();
            XorShift32 rng { seed };

            constexpr std::size_t kTotalCells = width_ * height_;
            const std::size_t totalHoles = static_cast<std::size_t>(static_cast<float>(kTotalCells) * rate);
            std::array<Position, kTotalCells> pos {};

            // Collect valid positions
            std::size_t validNum { 0 };
            for (std::size_t y = 0; y < height_; ++y) {
                for (std::size_t x = 0; x < width_; ++x) {
                    const Position currentPos { x, y };
                    if (pond_.getType(currentPos) == Type::Empty) { pos[validNum++] = currentPos; }
                }
            }

            // Shuffle available positions
            std::shuffle(pos.begin(), pos.begin() + validNum, rng);

            // Place holes
            const std::size_t holesToPlace = std::min(totalHoles, validNum);
            for (std::size_t n = 0; n < holesToPlace; ++n) {
                static_cast<void>(pond_.setType(pos[n], Type::Hole));
            } return validNum >= totalHoles;
        }

        void printPosition(Position pos) const { std::println("x={}, y={}", pos.x_, pos.y_); }

        void printCell(Cell cell) const {
            const Position pos = cell.getPosition();
            std::print("Type={} ", pond_.getType(pos));
            printPosition(pos);
        }

        void print(const bool useColors = true) const {
            std::println("Frozen Pond: [{}x{}]", width_, height_);
            std::array<char, kBufferSize_> buffer {};
            auto it = buffer.begin();
            for (std::size_t y = 0; y < height_; ++y) {
                for (std::size_t x = 0; x < width_; ++x) {
                    const Type type = pond_.getType(Position { x, y });
                    const std::string_view sv = getTypeString(type);
                    const auto remaining = static_cast<std::size_t>(std::distance(it, buffer.end()));
                    if (useColors) {
                        const auto idx = static_cast<std::size_t>(type);
                        const std::string_view color = idx < kTypeColors.size() ? kTypeColors[idx] : kTypeColors.back();
                        auto res = std::format_to_n(it, remaining, "{}{}{} ", color, sv, kReset_);
                        it = res.out;
                    } else {
                        auto res = std::format_to_n(it, remaining, "{} ", sv);
                        it = res.out;
                    }
                } if (it != buffer.end()) { *it++ = '\n'; }
            }
            const std::string_view gridView { buffer.data(), static_cast<std::size_t>(std::distance(buffer.begin(), it)) };
            std::print("{}{}{}", kCursorOff_, gridView, kCursorOn_);
        }
    };

} // namespace pond