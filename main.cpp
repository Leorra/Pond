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

static constexpr std::size_t kPondWidth = 40;
static constexpr std::size_t kPondHeight = 20;
static constexpr float kHolesRate = 0.005f;

static constexpr std::size_t kPathLength = kPondWidth * kPondHeight * 10;

int main() {
	pond::Pond<kPondWidth, kPondHeight> pond;
	std::random_device rd;
	const std::uint32_t seed = static_cast<std::uint32_t>(rd());

	pond::PondUtils pond_utils { pond };
	const bool success = pond_utils.populateHoles(kHolesRate, seed);
	if (!success) { std::println("[WARN]: Could not populate all the requested holes!"); }

	const std::size_t total_cells = kPondWidth * kPondHeight;
	const std::size_t holes_count = pond_utils.getHolesCount();
	const float holes_count_pct = static_cast<float>(holes_count) / static_cast<float>(kPondWidth * kPondHeight) * 100.0f;
	std::println("[INFO]: Frozen Pond: [{}x{}] {} total\n[INFO]: Holes count: [{}] {:.2f}%",
		kPondWidth, kPondHeight, total_cells, holes_count, holes_count_pct);

//	pond_utils.print(true);

	pond::Path<pond::Position, kPathLength> path;
	pond::QTable q_table { pond };
	static_assert(std::is_same_v<decltype(q_table), pond::QTable<pond::Pond<kPondWidth, kPondHeight>>>);

	// Create a simple path through the pond using the QTable
	bool path_created = q_table.createSinglePath(path);
	std::size_t path_size = path.size();
	std::println("[INFO]: Path created: Size: [{}], Terminal: [{}]", path_size, path_created);
	
	pond_utils.applyPath(path);
	pond_utils.print(true);

	return 0;
}