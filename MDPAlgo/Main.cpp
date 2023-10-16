#include "TSP.hpp"
#include <iostream>
#include "MazeSolver.hpp"
#include <chrono>

void TestTSP()
{
	std::vector<std::vector<int>> distance_matrix = {
	{0, 5, 4, 10},
	{5, 0, 8, 5},
	{4, 8, 0, 3},
	{10, 5, 3, 0}
	};
	auto result = TSP::solve(distance_matrix);
	std::cout << "best distance: " << result.best_distance << std::endl;
}

int main()
{
	MDP::MazeSolver ms({ 20, 20 }, { 1,1 }, MDP::FD_North);
	ms.AddObstacle({ 10, 10 }, MDP::FD_North);
	ms.AddObstacle({ 10, 17 }, MDP::FD_East);
	ms.AddObstacle({4, 17}, MDP::FD_South);
	ms.AddObstacle({ 18, 14 }, MDP::FD_West);
	ms.AddObstacle({ 18, 5 }, MDP::FD_West);
	ms.AddObstacle({ 12, 5 }, MDP::FD_North);

	auto start = std::chrono::system_clock::now();
	auto result2 = ms.GetOptimalOrderDP(false);
	auto end = std::chrono::system_clock::now();
	auto elapse = end - start;
	std::cout << "Duration taken: " << std::chrono::duration_cast<std::chrono::milliseconds>(elapse) << std::endl;
	std::cout << "result:" << result2.size() << std::endl;
	std::cout << result2 << std::endl;
	return 0;
}