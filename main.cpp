/*
======================================================
[+] A very serious the Frozen Pond problem project [+]
[+] C++ 23 Code Standard, SoA SIMD friendly Design [+]
[+] https://github.com/Leorra                      [+]
======================================================
*/

#include <print>
#include <random>

#include "Pond.hpp"
#include "PondUtils.hpp"
#include "QTable.hpp"

using namespace pond;

static constexpr std::size_t kPondWidth = 30;
static constexpr std::size_t kPondHeight = 20;
static constexpr float kHolesRate = 0.5f;

int main() {
	Pond<kPondWidth, kPondHeight> pond;
	std::random_device random_device;

	PondUtils pond_utils { pond };
	const bool success = pond_utils.populateHoles(kHolesRate, random_device);
	if (!success) { std::println("[WARNING]: Could not place all requested holes!"); }

	const std::size_t total_cells = kPondWidth * kPondHeight;
	const std::size_t holes_count = pond_utils.getHolesCount();
	const float holes_pct = static_cast<float>(holes_count) / static_cast<float>(kPondWidth * kPondHeight) * 100.0f;
	std::println("[INFO]: Frozen Pond: [{}x{}] {} total\n[INFO]: Holes count: [{}] {:.2f}%",
		kPondWidth, kPondHeight, total_cells, holes_count, holes_pct);	

	pond_utils.print(true);

	QTable<Pond<kPondWidth, kPondHeight>> q_table { pond };

	return 0;
}