#pragma once
#include <vector>

namespace TSP {
	struct TSP_Result {
		std::vector<int> permutation;
		int best_distance;
	};

	TSP_Result solve(const std::vector<std::vector<int>>& distance_matrix);
}