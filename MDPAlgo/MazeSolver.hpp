#pragma once
#include <Windows.h>
#include "FieldObjects.hpp"
#include <unordered_map>

namespace MDP {
	class Grid {

	public:
		Grid(const POINT& size);

		void AddObstacle(const POINT& loc, FaceDirection dir);
		void AddObstacle(const SFieldObject& obj);
		std::vector<std::vector<ObjectState>> GetViewObstaclePositions(bool retrying);
		bool Reachable(const POINT& xy, bool turn = false, bool preTurn = false);
		std::vector<SFieldObject> GetObjects() const;
	private:
		POINT mSize;
		std::vector<SFieldObject> mObjects;

		bool IsValidCoord(const POINT& xy);
	};


	struct PathData : ObjectState {

		PathData(const ObjectState& s) :
			ObjectState(s)
		{
		}
	};

	struct Neighbor : ObjectState {
		int cost;

		Neighbor(const POINT& loc, FaceDirection d, int cost) :
			ObjectState(loc, d), cost(cost)
		{
		}
	};

	class MazeSolver {
		
	public:
		MazeSolver(const POINT& grid_size, const POINT& robot, FaceDirection robot_dir, bool big_turn = false);

		MazeSolver& AddObstacle(const POINT& loc, FaceDirection dir);
		MazeSolver& AddObstacle(const SFieldObject& obj);
		std::vector<SFieldObject> GetObstacles() const;
		std::vector<ObjectState> GetOptimalOrderDP(bool retrying);

	private:
		bool mBigTurn;
		Grid mGrid;
		std::shared_ptr<FieldRobot> mRobot;
		std::unordered_map<FieldStartEnd, std::vector<PathData>> path_table;
		std::unordered_map<FieldStartEnd, int> cost_table;
		struct WRT_BIG_TURNS {
			int left_wheel;
			int right_wheel;
		}turn_wrt_big_turns[2];

		void GeneratePathCost(const std::vector<ObjectState>& states);
		void DoAStarSearch(const ObjectState& start, const ObjectState& end);
		std::vector<Neighbor> GetNeighbors(const ObjectState& s);

		int GetSafeCost(const POINT& xy);
		void GenerateCombination(const std::vector<std::vector<ObjectState>>& view_pos, 
			std::size_t index, std::vector<int>& current, std::vector<std::vector<int>>& result,
			std::size_t& iteration_left);

		void RecordPath(const ObjectState& start, const ObjectState& end,
			const std::unordered_map<ObjectState, ObjectState>& parent, int distance);
	};
}