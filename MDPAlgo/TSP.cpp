#include "TSP.hpp"
#include <set>
#include <map>
#include <algorithm>
#include <iterator>
#include <iostream>

namespace TSP {
	struct memo_struct {
		int ni;
		std::set<int> n;

		bool operator<(const memo_struct& rhs) const
		{
			if (this->ni < rhs.ni)
				return true;
			else if (this->ni > rhs.ni)
				return false;
			return this->n < rhs.n;
		}

		bool operator==(const memo_struct& rhs) const
		{
			return this->ni == rhs.ni &&
				this->n == rhs.n;
		}
	};

	std::map<memo_struct, int> memo;

	std::set<int> difference(const std::set<int>& s1, const std::set<int>& s2) 
	{
		std::set<int> result;
		std::set_difference(s1.begin(), s1.end(), s2.begin(), s2.end(),
			std::inserter(result, result.end()));
		return result;
	}

	int dist(const std::vector<std::vector<int>>& distance_matrix, int ni, const std::set<int>& N)
	{
		if (N.empty())
			return distance_matrix[ni][0];

		struct temp_cost {
			int nmin;
			int cost;
		};
		std::vector<temp_cost> cost;
		cost.resize(N.size());
		int index = 0;
		for (auto& nj : N) {
			cost[index++] = { nj, distance_matrix[ni][nj] + dist(distance_matrix, nj, difference(N, { nj })) };
		}
		std::sort(cost.begin(), cost.end(), [](const temp_cost& l, const temp_cost& r) {
			return l.cost < r.cost;
		});
		auto& lowest_cost = cost[0];
		memo_struct t{ ni, N };
#if LogTSP >= 1
		std::cout << "[TSP-lowest, CostSize:" << cost.size() <<"]nmin:" << lowest_cost.nmin << "cost:" << lowest_cost.cost << std::endl;
#endif
		memo.insert(std::make_pair(t, lowest_cost.nmin));

		return lowest_cost.cost;
	}

	TSP_Result solve(const std::vector<std::vector<int>>& distance_matrix)
	{
		memo.clear();
		auto N_Size = distance_matrix.size();
		std::set<int> N;
		for (int i = 1; i < N_Size; i++) N.insert(i);

		TSP_Result result;
		result.best_distance = dist(distance_matrix, 0, N);
#if LogTSP >= 1
		std::cout << "[TSP-memo]result.best_distance:" << result.best_distance << std::endl;
#endif
		int ni = 0;
		result.permutation.push_back(0);

		while (!N.empty()) {
			ni = memo[{ ni, N }];
#if LogTSP >= 1
			std::cout << "[TSP-memo]ni:" << ni << std::endl;
#endif
			result.permutation.push_back(ni);
			N = difference(N, { ni });
		}
		return result;
	}
}