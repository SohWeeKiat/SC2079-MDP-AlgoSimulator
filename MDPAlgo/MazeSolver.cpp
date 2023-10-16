#include "MazeSolver.hpp"
#include <algorithm>
#include <iterator>
#include <queue>
#include <set>
#include "TSP.hpp"
#include <iostream>
#include "Utils.hpp"
#include "Config.hpp"

namespace MDP {
	enum dist_type {
		dt_Ln,
		dt_euclidean,
	};

	const struct MoveDirection {
		POINT dxdy;
		FaceDirection direction;
	}MOVE_DIRECTION[4]{
		{ {1,0}, FaceDirection::FD_East},
		{ {-1,0}, FaceDirection::FD_West},
		{ {0,1}, FaceDirection::FD_North},
		{ {0,-1}, FaceDirection::FD_South},
	};

	/*struct WRT_BIG_TURNS {
		int left_wheel;
		int right_wheel;
	}turn_wrt_big_turns[2]{
		{3 * Config::get().Get_TURN_RADIUS(), Config::get().Get_TURN_RADIUS()},
		{4 * Config::get().Get_TURN_RADIUS(), 2 * Config::get().Get_TURN_RADIUS()},
	};*/

	int compute_dist(int x1, int y1, int x2, int y2, dist_type type = dist_type::dt_Ln)
	{
		int horizontal_distance = x1 - x2;
		int vertical_distance = y1 - y2;

		switch (type) {
		case dist_type::dt_Ln:
			return abs(horizontal_distance) + abs(vertical_distance);
		case dist_type::dt_euclidean:
			return sqrt(pow(horizontal_distance, 2) + pow(vertical_distance, 2));
		default:
			return 0;
		}
	}

	int rotation_cost(FaceDirection fd1, FaceDirection fd2)
	{
		auto get_direction_cost = [](FaceDirection fd) {
			switch (fd) {
			case FaceDirection::FD_North: return 0;
			case FaceDirection::FD_East: return 2;
			case FaceDirection::FD_South: return 4;
			case FaceDirection::FD_West: return 6;
			}
		};
		int d1 = get_direction_cost(fd1), d2 = get_direction_cost(fd2);
		int diff = abs(d1 - d2);
		return min(diff, 8 - diff);
	}

	int countBit(int val)
	{
		if (val == 0) return 0;
		else
			return (val & 1) + countBit(val >> 1);
	}

	std::vector<int> GetVisitOptions(int n)
	{
		std::vector<int> output;
		int max_val = pow(2, n);
		for (int i = 0; i < max_val;i++)
			output.push_back(i);
		std::sort(output.begin(), output.end(), [](const int& l, const int& r) {
			return countBit(l) > countBit(r);
		});
		return output;
	}

	Grid::Grid(const POINT& size) : 
		mSize(size)
	{

	}

	bool Grid::IsValidCoord(const POINT& xy)
	{
		if (xy.x < 1 || xy.x >= this->mSize.x - 1 ||
			xy.y < 1 || xy.y >= this->mSize.y - 1)
			return false;
		return true;
	}

	bool Grid::Reachable(const POINT& xy, bool turn, bool preTurn)
	{
		if (!this->IsValidCoord(xy))
			return false;
		for (auto& obj : this->mObjects) {
			if (obj->GetLoc().x == 4 && obj->GetLoc().y <= 4 &&
				xy.x < 4 && xy.y < 4)
				continue;
			//Must be at least 4 units away in total (x+y)
			if (abs(obj->GetLoc().x - xy.x) + abs(obj->GetLoc().y - xy.y) >= 4)
				continue;
			if (turn) {
				if (max(abs(obj->GetLoc().x - xy.x), abs(obj->GetLoc().y - xy.y)) < (Config::get().Get_EXPANDED_CELL() * 2 + 1))
					return false;
			}
			if (preTurn) {
				if (max(abs(obj->GetLoc().x - xy.x), abs(obj->GetLoc().y - xy.y)) < (Config::get().Get_EXPANDED_CELL() * 2 + 1))
					return false;
			}
			else {
				if (max(abs(obj->GetLoc().x - xy.x), abs(obj->GetLoc().y - xy.y)) < 2)
					return false;
			}
		}
		return true;
	}

	std::vector<std::vector<ObjectState>> Grid::GetViewObstaclePositions(bool retrying)
	{
		std::vector<std::vector<ObjectState>> output;
		for (auto& obj : this->mObjects) {
			if (obj->GetDirection() == FaceDirection::FD_None) continue;
			auto res = obj->GetViewState(retrying);
			std::vector<ObjectState> new_list;
			for (auto& s : res) {
				if (this->Reachable(s.m_location))
					new_list.push_back(s);
			}
			output.push_back(new_list);
		}
		return output;
	}

	void Grid::AddObstacle(const POINT& loc, FaceDirection dir)
	{
		if (loc.x < 0 || loc.x >= this->mSize.x ||
			loc.y < 0 || loc.y >= this->mSize.y)
			return;
		this->mObjects.push_back(std::make_shared<FieldBlock>(loc, dir));
	}

	void Grid::AddObstacle(const SFieldObject& obj)
	{
		this->mObjects.push_back(obj);
	}

	std::vector<SFieldObject> Grid::GetObjects() const
	{
		return this->mObjects;
	}

	MazeSolver::MazeSolver(const POINT& grid_size, const POINT& robot, FaceDirection robot_dir,
		bool big_turn) : mGrid(grid_size),
		mRobot(std::make_shared<FieldRobot>(robot, robot_dir)),
		mBigTurn(big_turn), turn_wrt_big_turns{
			//{3 * Config::get().Get_TURN_RADIUS(), Config::get().Get_TURN_RADIUS()},
			{Config::get().Get_LEFTWHEEL() * Config::get().Get_TURN_RADIUS(), Config::get().Get_RIGHTWHEEL() * Config::get().Get_TURN_RADIUS()},
			{4 * Config::get().Get_TURN_RADIUS(), 2 * Config::get().Get_TURN_RADIUS()},
		}
	{
	}

	MazeSolver& MazeSolver::AddObstacle(const POINT& loc, FaceDirection dir)
	{
		this->mGrid.AddObstacle(loc, dir);
		return *this;
	}

	MazeSolver& MazeSolver::AddObstacle(const SFieldObject& obj)
	{
		this->mGrid.AddObstacle(obj);
		return *this;
	}

	std::vector<SFieldObject> MazeSolver::GetObstacles() const
	{
		return this->mGrid.GetObjects();
	}

	std::vector<ObjectState> MazeSolver::GetOptimalOrderDP(bool retrying)
	{
		std::vector<ObjectState> optimal_path;
		int distance = 0x7FFFFFFF;
		auto all_pos = this->mGrid.GetViewObstaclePositions(retrying);
		//std::cout << "all_pos:" << all_pos.size() << std::endl;

		//auto visit_options = ;
		//std::cout << "visit_options:" << visit_options << std::endl;
		for (auto& op : GetVisitOptions(all_pos.size()))
		{
			std::vector<ObjectState> items { this->mRobot->GetState() };
			std::vector<std::vector<ObjectState>> CurViewPos;
			for (std::size_t i = 0; i < all_pos.size(); i++)
			{
				//if robot is visiting
				if ((op >> i) & 1)
				{
					auto& view_pos = all_pos[i];
					std::copy(view_pos.begin(), view_pos.end(),
						std::back_inserter(items));
					//std::cout << "[view_pos]" << view_pos.size() << std::endl;
					CurViewPos.push_back(view_pos);
					//std::cout << "visiting " << op << std::endl;
				}
			}
			/*for (auto& vp : CurViewPos) {
				std::cout << "[vp]" << vp << std::endl;
			}*/
			//std::cout << "[CurViewPos]" << CurViewPos.size() << std::endl;
			this->GeneratePathCost(items);
			std::vector<int> current;
			std::vector<std::vector<int>> combination;
			std::size_t iterations = Config::get().Get_ITERATIONS();
			this->GenerateCombination(CurViewPos, 0, current, combination, iterations);
			//std::cout << "[combination]" << combination.size() << std::endl;
			for (auto& c : combination)
			{
				std::vector<int> visited_candidates{0};
				std::size_t cur_index = 1;
				int fixed_cost = 0;
				for (std::size_t index = 0; index < CurViewPos.size(); index++) 
				{
					auto& view_pos = CurViewPos[index];
					visited_candidates.push_back(cur_index + c[index]);
					fixed_cost += view_pos[c[index]].penalty;
					cur_index += view_pos.size();
				}
				//std::cout << "[visited_candidates]" << visited_candidates << std::endl;
				//std::cout << "[CurViewPos]" << CurViewPos.size() << std::endl;
				std::vector<std::vector<int>> cost_np;
				cost_np.resize(visited_candidates.size());
				for (auto& row : cost_np) row.resize(visited_candidates.size());
				for (std::size_t y = 0; y < visited_candidates.size(); y++)
				{
					for (std::size_t x = y + 1; x < visited_candidates.size(); x++)
					{
						ObjectState u = items[visited_candidates[y]];
						ObjectState v = items[visited_candidates[x]];
						if (this->cost_table.find({ u , v }) != this->cost_table.end())
							cost_np[y][x] = this->cost_table[{u, v}];
						else
							cost_np[y][x] = 0x7FFFFFFF;
						cost_np[x][y] = cost_np[y][x];
					}
				}
				for (std::size_t y = 0; y < cost_np.size(); y++) {
					/*for (std::size_t x = 0; x < cost_np[y].size(); x++) {
						std::cout << "y(" << y << ", " << x << "): " << cost_np[y][x] << std::endl;
					}*/
					cost_np[y][0] = 0;
				}
				//std::cout << "Before TSP.solve" << std::endl;
				auto result = TSP::solve(cost_np);
				//std::cout << "TSP.solve dist: " << result.best_distance << std::endl;
				//std::cout << "fixed_cost: " << fixed_cost << std::endl;
				if (result.best_distance + fixed_cost >= distance)
					continue;
				//std::cout << "result.permutation: " << result.permutation.size() << result.permutation << std::endl;
				optimal_path = { items[0] };
				distance = result.best_distance + fixed_cost;
				for (std::size_t i = 0; i < result.permutation.size() - 1; i++)
				{
					auto& from_item = items[visited_candidates[result.permutation[i]]];
					auto& to_item = items[visited_candidates[result.permutation[i + 1]]];

					auto cur_path = this->path_table[{from_item, to_item}];
					for (std::size_t j = 1; j < cur_path.size(); j++)
					{
						optimal_path.push_back(cur_path[j]);
					}
				}
			}
			if (!optimal_path.empty()) {
				break;
			}
		}
		return optimal_path;
	}

	void MazeSolver::GeneratePathCost(const std::vector<ObjectState>& states)
	{
		for (std::size_t i = 0; i < states.size() - 1; i++) 
		{
			for (std::size_t j = i + 1; j < states.size(); j++) 
			{
				this->DoAStarSearch(states[i], states[j]);
			}
		}
	}

	void MazeSolver::DoAStarSearch(const ObjectState& start, const ObjectState& end)
	{
		struct temp_heap_struct : ObjectState {
			int f_distance;

			temp_heap_struct(const ObjectState& s, int f_distance = 0) :
				ObjectState(s), f_distance(f_distance)
			{
			}

			bool operator<(const temp_heap_struct& rhs) const
			{
				if (this->f_distance < rhs.f_distance)
					return true;
				else if (this->f_distance > rhs.f_distance)
					return false;

				if (this->m_location.x < rhs.m_location.x)
					return true;
				else if (this->m_location.x > rhs.m_location.x)
					return true;

				if (this->m_location.y < rhs.m_location.y)
					return true;
				else if (this->m_location.y > rhs.m_location.y)
					return false;
				

				if (this->m_Fd < rhs.m_Fd)
					return true;
				return false;
			}

			bool operator>(const temp_heap_struct& rhs) const
			{
				if (this->f_distance > rhs.f_distance)
					return true;
				else if (this->f_distance < rhs.f_distance)
					return false;

				if (this->m_location.x > rhs.m_location.x)
					return true;
				else if (this->m_location.x < rhs.m_location.x)
					return false;

				if (this->m_location.y > rhs.m_location.y)
					return true;
				else if (this->m_location.y < rhs.m_location.y)
					return false;

				if (this->m_Fd > rhs.m_Fd)
					return true;
				return false;
			}
		};

		FieldStartEnd se{ start, end };
		if (this->path_table.find(se) != this->path_table.end())
			return;
		std::priority_queue<temp_heap_struct, std::vector<temp_heap_struct>,
			std::greater<temp_heap_struct>> pq;
		std::set<ObjectState> visited;
		std::unordered_map<ObjectState, int> g_distance;
		std::unordered_map<ObjectState, ObjectState> parent;
		g_distance[start] = 0;
		pq.push({ start, (int)compute_dist(start.m_location.x, start.m_location.y, end.m_location.x, end.m_location.y)});

		while (!pq.empty())
		{
			auto item = pq.top();
			pq.pop();
			if (visited.find(item) != visited.end())
				continue;
			else if (end == ObjectState(item)) {
				//record path
				return this->RecordPath(start, end, parent, g_distance[item]);
			}
			visited.insert(item);
			int cur_distance = g_distance[item];
			for (auto& n : this->GetNeighbors(item))
			{
				if (visited.find(n) != visited.end())
					continue;
				int move_cost = rotation_cost(n.m_Fd, item.m_Fd) * Config::get().Get_TURN_FACTOR() + 1 + n.cost;
				int next_cost = cur_distance + move_cost +
					compute_dist(n.m_location.x, n.m_location.y,
						end.m_location.x, end.m_location.y);
				if (g_distance.find(n) == g_distance.end() ||
					g_distance[n] > cur_distance + move_cost) {
					g_distance[n] = cur_distance + move_cost;
					parent[n] = item;
					pq.push({ n,next_cost });
				}
			}
		}
	}

	std::vector<Neighbor> MazeSolver::GetNeighbors(const ObjectState& s)
	{
		std::vector<Neighbor> result;
		for (auto& d : MOVE_DIRECTION) 
		{
			if (d.direction == s.m_Fd)
			{
				auto CheckReachableAndAppend = [this, &result](const POINT& cur, const POINT& displacement, 
					FaceDirection d) {
					POINT NewLoc = { cur.x + displacement.x, cur.y + displacement.y };
					if (this->mGrid.Reachable(NewLoc)) {
						int cost = this->GetSafeCost(NewLoc);
						result.push_back({ NewLoc, d, cost });
					}
				};
				CheckReachableAndAppend(s.m_location, d.dxdy, d.direction);
				CheckReachableAndAppend(s.m_location, { -d.dxdy.x, -d.dxdy.y }, d.direction);
			}
			else
			{
				//consider 8 cases
				auto CheckReachableAndAppend = [this, &result](const POINT& cur, const POINT& displacement,
					FaceDirection d) {
					POINT NewLoc = { cur.x + displacement.x, cur.y + displacement.y };
					if (this->mGrid.Reachable(NewLoc, true) && this->mGrid.Reachable(cur, false, true)) {
						int cost = this->GetSafeCost(NewLoc);
						result.push_back({ NewLoc, d, cost + 10 });
					}
				};
				int bigger_change = this->turn_wrt_big_turns[this->mBigTurn].left_wheel;
				int smaller_change = this->turn_wrt_big_turns[this->mBigTurn].right_wheel;
				switch (s.m_Fd)//current facing direction
				{
				//north > east/west
				case FaceDirection::FD_North:
				{
					if (d.direction == FaceDirection::FD_East)
					{
						CheckReachableAndAppend(s.m_location, { bigger_change, smaller_change }, d.direction);
						CheckReachableAndAppend(s.m_location, { -smaller_change, -bigger_change }, d.direction);
					}
					else if (d.direction == FaceDirection::FD_West)
					{
						CheckReachableAndAppend(s.m_location, { smaller_change, -bigger_change }, d.direction);
						CheckReachableAndAppend(s.m_location, { -bigger_change, smaller_change }, d.direction);
					}
				}
				break;
				//east > north/south
				case FaceDirection::FD_East:
				{
					if (d.direction == FaceDirection::FD_North)
					{
						CheckReachableAndAppend(s.m_location, { smaller_change, bigger_change }, d.direction);
						CheckReachableAndAppend(s.m_location, { -bigger_change, -smaller_change }, d.direction);
					}else if (d.direction == FaceDirection::FD_South)
					{
						CheckReachableAndAppend(s.m_location, { smaller_change, -bigger_change }, d.direction);
						CheckReachableAndAppend(s.m_location, { -bigger_change, smaller_change }, d.direction);
					}
				}
				break;
				//south > east/west
				case FaceDirection::FD_South:
				{
					if (d.direction == FaceDirection::FD_East)
					{
						CheckReachableAndAppend(s.m_location, { bigger_change, -smaller_change }, d.direction);
						CheckReachableAndAppend(s.m_location, { -smaller_change, bigger_change }, d.direction);
					}
					else if (d.direction == FaceDirection::FD_West)
					{
						CheckReachableAndAppend(s.m_location, { -bigger_change, -smaller_change }, d.direction);
						CheckReachableAndAppend(s.m_location, { smaller_change, bigger_change }, d.direction);
					}
				}
				break;
				//west > south/north
				case FaceDirection::FD_West:
				{
					if (d.direction == FaceDirection::FD_South)
					{
						CheckReachableAndAppend(s.m_location, { -smaller_change, -bigger_change }, d.direction);
						CheckReachableAndAppend(s.m_location, { bigger_change, smaller_change }, d.direction);
					}
					else if (d.direction == FaceDirection::FD_North)
					{
						CheckReachableAndAppend(s.m_location, { -smaller_change, bigger_change }, d.direction);
						CheckReachableAndAppend(s.m_location, { bigger_change, -smaller_change }, d.direction);
					}
				}
				break;
				}
			}
		}
		return result;
	}

	int MazeSolver::GetSafeCost(const POINT& xy)
	{
		for (auto& obj : this->mGrid.GetObjects())
		{
			if (abs(obj->GetLoc().x - xy.x) == 2 && abs(obj->GetLoc().y - xy.y) == 2)
				return Config::get().Get_SAFE_COST();
			if (abs(obj->GetLoc().x - xy.x) == 1 && abs(obj->GetLoc().y - xy.y) == 2)
				return Config::get().Get_SAFE_COST();
			if (abs(obj->GetLoc().x - xy.x) == 2 && abs(obj->GetLoc().y - xy.y) == 1)
				return Config::get().Get_SAFE_COST();
		}
		return 0;
	}

	void MazeSolver::GenerateCombination(const std::vector<std::vector<ObjectState>>& view_pos, 
		std::size_t index, std::vector<int>& current, std::vector<std::vector<int>>& result, 
		std::size_t& iteration_left)
	{
		if (index == view_pos.size()) {
			//std::cout << "[MazeSolver::GenerateCombination]pushing current" << std::endl;
			result.push_back(current);
			return;
		}
		if (iteration_left == 0)
			return;
		iteration_left -= 1;
		//std::cout << "[MazeSolver::GenerateCombination]view_pos[index].size():" 
			//<< view_pos[index].size() << std::endl;
		for (std::size_t i = 0; i < view_pos[index].size(); i++) {
			current.push_back(i);
			this->GenerateCombination(view_pos, index + 1, current, result, iteration_left);
			current.pop_back();
		}
	}

	void MazeSolver::RecordPath(const ObjectState& start, const ObjectState& end,
		const std::unordered_map<ObjectState, ObjectState>& parent, int distance)
	{
		this->cost_table[{start, end}] = distance;
		this->cost_table[{end, start}] = distance;

		std::vector<PathData> path;
		PathData temp{ end };
		while (parent.find(temp) != parent.end()) {
			path.push_back(temp);
			temp = PathData(parent.at(temp));
		}
		path.push_back(temp);

		auto reversed = path;
		std::reverse(reversed.begin(), reversed.end());
		this->path_table[{start, end}] = reversed;
		this->path_table[{end, start}] = path;
	}
}