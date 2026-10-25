/*] ============================================== [*]
[*] A very serious the Frozen Pond problem project [*]
[*] C++ 23 Code Standard, SoA SIMD friendly Design [*]
[*]     (^_^) https://github.com/Leorra (^_^)      [*]
[*] ============================================== [*/

#include <cstdint>
#include <print>
#include <random>
#include <type_traits>

#include "Path.hpp"
#include "Pond.hpp"
#include "PondUtils.hpp"
#include "QTable.hpp"

static constexpr std::size_t kPondWidth = 25;
static constexpr std::size_t kPondHeight = 25;
static constexpr float kHolesRate = 0.15f;

static constexpr std::size_t kPathLength = kPondWidth * kPondHeight * 10;
static constexpr std::size_t kMaxEpisodes = 1'000'000'000;
static constexpr std::size_t kMaxGoalEpisodes = 10'000;
static constexpr std::size_t kPeriodEpisodes = 1'000'000;


static constexpr float kEpsilonStart = 1.0f; // Exploration rate at the first episode
static constexpr float kEpsilonEnd = 0.0f;   // Exploration rate at the last episode

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
	pond::QTable q_table { pond, static_cast<std::uint32_t>(rd()) };
	static_assert(std::is_same_v<decltype(q_table), pond::QTable<pond::Pond<kPondWidth, kPondHeight>>>);

	std::size_t goal_episodes = 0;
	std::size_t hole_episodes = 0;
	std::size_t truncated_episodes = 0;
	std::size_t first_goal_episode = 0;

	for (std::size_t episode = 1; episode <= kMaxEpisodes; ++episode) {
		q_table.setSeed(static_cast<std::uint32_t>(rd())); // Randomize the seed for each episode

		// Linear epsilon decay from kEpsilonStart (first episode) to kEpsilonEnd (last episode)
		const float progress = static_cast<float>(episode - 1) / static_cast<float>(kMaxEpisodes - 1);
		static_cast<void>(q_table.setEpsilon(kEpsilonStart + (kEpsilonEnd - kEpsilonStart) * progress));

		const bool terminal = q_table.createSinglePath(path);

		// Every episode updates the table: goal, hole and truncated alike.
		static_cast<void>(q_table.updateTable(path));
		if (!terminal) { ++truncated_episodes; continue; }
		if (pond.getType(path[path.size() - 1]) == pond::Type::Goal) {
			if (goal_episodes == 0) { first_goal_episode = episode; }
			++goal_episodes;
			best_path = path;
			if (goal_episodes >= kMaxGoalEpisodes) { break; }
		} else { ++hole_episodes; }
		if (episode % kPeriodEpisodes == 0) {
			std::print("[INFO]: Episode [{}] Hole: [{}] Truncated: [{}] Goal: [{}]\r",
				episode, hole_episodes, truncated_episodes, goal_episodes);
		}
	}

	std::println("[INFO]: Episodes: [{}] Goal: [{}] Hole: [{}] Truncated: [{}]",
		goal_episodes + hole_episodes + truncated_episodes, goal_episodes, hole_episodes, truncated_episodes);

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