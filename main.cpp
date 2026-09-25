#include <print>
#include <random>
#include "Pond.hpp"
#include "PondUtils.hpp"

using namespace pond;

static constexpr std::size_t kPondWidth = 40;
static constexpr std::size_t kPondHeight = 20;
static constexpr float kHolesRate = 0.25f;

int main() {
	Pond<kPondWidth, kPondHeight> pond;

	std::random_device random_device;

	PondUtils utils { pond };
	const bool success = utils.populateHoles(kHolesRate, random_device); utils.print();
	if (!success) { std::println("Warning: Could not place all requested holes."); }

	return 0;
}