/*] ============================================== [*]
[*] A very serious the Frozen Pond problem project [*]
[*] C++ 23 Code Standard, SoA SIMD friendly Design [*]
[*]     (^_^) https://github.com/Leorra (^_^)      [*]
[*] ============================================== [*/

#include <cstdint>
#include <print>
#include <random>

#include "Path.hpp"
#include "Pond.hpp"
#include "PondUtils.hpp"
#include "QTable.hpp"

static constexpr std::size_t kPondWidth = 40;
static constexpr std::size_t kPondHeight = 20;
static constexpr float kHolesRate = 0.25f;

static constexpr std::size_t kPathLength = kPondWidth * kPondHeight * 10;
static constexpr std::size_t kMaxEpisodes = 1'000'000'000;

int main() {
	pond::Pond<kPondWidth, kPondHeight> pond;
	std::random_device rd;
	const std::uint32_t seed = static_cast<std::uint32_t>(rd());

	pond::PondUtils pond_utils { pond };
	const bool success = pond_utils.populateHoles(kHolesRate, seed);
	if (!success) { std::println("[WARN]: Could not populate all the requested holes!"); }

	const std::size_t total_cells = kPondWidth * kPondHeight;
	const std::size_t holes_count = pond_utils.getHolesCount();
	const float holes_count_pct = static_cast<float>(holes_count) / static_cast<float>(total_cells) * 100.0f;
	std::println("[INFO]: Frozen Pond: [{}x{}] {} total\n[INFO]: Holes count: [{}] {:.2f}%",
		kPondWidth, kPondHeight, total_cells, holes_count, holes_count_pct);

	pond_utils.print(true);

	pond::Path<pond::Position, kPathLength> path;
	pond::Path<pond::Position, kPathLength> best_path; // Last episode that reached the goal
	pond::QTable q_table { pond };
	static_assert(std::is_same_v<decltype(q_table), pond::QTable<pond::Pond<kPondWidth, kPondHeight>>>);

	std::size_t goal_episodes = 0;
	std::size_t hole_episodes = 0;
	std::size_t truncated_episodes = 0;
	std::size_t first_goal_episode = 0;

	for (std::size_t episode = 1; episode <= kMaxEpisodes; ++episode) {
		const bool terminal = q_table.createSinglePath(path);
		if (!terminal) { ++truncated_episodes; continue; }

		// Every completed episode (goal or hole) updates the table, including the successful one.
		static_cast<void>(q_table.updateTable(path));

		if (pond.getType(path[path.size() - 1]) == pond::Type::Goal) {
			if (goal_episodes == 0) { first_goal_episode = episode; }
			++goal_episodes;
			best_path = path;
//			std::println("Goal reached at episode [{}], path size: [{}]", episode, path.size());
//			break;
		} else { ++hole_episodes; }
	}

	std::println("[INFO]: Episodes: [{}] Goal: [{}] Hole: [{}] Truncated: [{}]",
		kMaxEpisodes, goal_episodes, hole_episodes, truncated_episodes);

	if (goal_episodes == 0) {
		std::println("[WARN]: Goal was never reached (it may be unreachable with this hole layout).");
		return 0;
	}

	std::println("[INFO]: First goal at episode [{}], last goal path size: [{}]",
		first_goal_episode, best_path.size());

	pond_utils.applyPath(best_path);
	pond_utils.print(true);

	return 0;
}