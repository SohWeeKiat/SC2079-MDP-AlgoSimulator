#include "FieldObjects.hpp"
#include <cassert>
#include <iostream>
#include "Config.hpp"
#include <string>

namespace MDP {

	bool is_valid(const POINT& center)
	{
		return center.x > 0 && center.y > 0 &&
			center.x < Config::get().Get_WIDTH_BUFFER() - 1 && center.y < Config::get().Get_HEIGHT_BUFFER() - 1;
	}

	std::ostream& operator<<(std::ostream& os, const FaceDirection& obj)
	{
		switch (obj){
		case FaceDirection::FD_None: os << "FD_None"; break;
		case FaceDirection::FD_North: os << "FD_North"; break;
		case FaceDirection::FD_East: os << "FD_East"; break;
		case FaceDirection::FD_South: os << "FD_South"; break;
		case FaceDirection::FD_West: os << "FD_West"; break;
		}
		return os;
	}

	std::ostream& operator<<(std::ostream& os, ObjectState& obj)
	{
		std::cout << "{x:" << obj.m_location.x << ", y:" << obj.m_location.y << ", fd:" << obj.m_Fd << "}";
		return os;
	}

	std::ostream& operator<<(std::ostream& os, std::vector<ObjectState>& obj)
	{
		std::cout << "[";
		for (auto& o : obj)
			std::cout << o << ",";
		std::cout << "]";
		return os;
	}

	FieldObject::FieldObject(const POINT& start_loc, const POINT& dimen,
		FaceDirection fd) :
		ObjectState(start_loc, fd), m_dimen(dimen)
	{
		//must be odd dimension
		assert(dimen.x % 2 == 1);
		assert(dimen.y % 2 == 1);
	}

	void FieldObject::Update(const POINT& loc, FaceDirection fd)
	{
		this->m_location = loc;
		this->m_Fd = fd;
	}

	void FieldObject::UpdateSnapshotID(int ID)
	{
		this->snapshot_id = ID;
	}

	bool FieldObject::IsInRect(const POINT& loc)
	{
		int dim_x = this->m_dimen.x / 2;
		int dim_y = this->m_dimen.y / 2;
		return loc.x >= this->m_location.x - dim_x &&
			loc.x <= this->m_location.x + dim_x &&
			loc.y <= this->m_location.y + dim_y &&
			loc.y >= this->m_location.y - dim_y;
	}

	void FieldObject::ChangeDirection()
	{
		if (this->m_Fd >= FaceDirection::FD_West)
			this->m_Fd = FaceDirection::FD_None;
		else
			this->m_Fd = static_cast<FaceDirection>(static_cast<int>(this->m_Fd) + 1);
	}

	std::vector<POINT> FieldObject::GetRectPoints()
	{
		std::vector<POINT> result;
		int dim_x = this->m_dimen.x / 2;
		int dim_y = this->m_dimen.y / 2;
		int startx = this->m_location.x - dim_x;
		int starty = this->m_location.y - dim_y;

		for (int x = startx; x < startx + this->m_dimen.x; x++)
		{
			for (int y = starty; y < starty + this->m_dimen.y; y++)
			{
				result.push_back({ x,y });
			}
		}
		return result;
	}

	FaceDirection FieldObject::GetDirection() const
	{
		return this->m_Fd;
	}

	POINT FieldObject::GetLoc() const
	{
		return this->m_location;
	}

	int FieldObject::GetSnapshotID() const
	{
		return this->snapshot_id;
	}

	ObjectState& FieldObject::GetState()
	{
		return *this;
	}

	std::vector<ObjectState> FieldObject::GetViewState(bool retrying)
	{
		return {};
	}

	/**********************************/

	FieldBlock::FieldBlock(const POINT& start_loc, FaceDirection fd, int obstacle_id) :
		FieldObject(start_loc, { 1,1 }, fd)
	{
		this->snapshot_id = obstacle_id;
	}


	GridBoxType FieldBlock::GetGridBoxType(const POINT& loc)
	{
		if (!this->IsInRect(loc))
			return GridBoxType::GBT_Invalid;
		switch (this->m_Fd)	{
		case FaceDirection::FD_None: return GridBoxType::GBT_None;
		case FaceDirection::FD_North: return GridBoxType::GBT_North;
		case FaceDirection::FD_East: return GridBoxType::GBT_East;
		case FaceDirection::FD_South: return GridBoxType::GBT_South;
		case FaceDirection::FD_West: return GridBoxType::GBT_West;
		default:break;
		}
		return GridBoxType::GBT_None;
	}

	std::vector<ObjectState> FieldBlock::GetViewState(bool retrying)
	{
		std::vector<ObjectState> output;
		auto CheckAndAdd = [&output](const POINT& loc, FaceDirection fd, int snapshot_id = -1, int penalty = 0) {
			if (is_valid(loc)) output.push_back(ObjectState(loc, fd, penalty, snapshot_id));
		};

		switch (this->m_Fd) {
		case FaceDirection::FD_North:
			if (!retrying) {
				//Or (x, y + 3)
				CheckAndAdd({ this->m_location.x, this->m_location.y + 1 + Config::get().Get_EXPANDED_CELL() * 2 }, FaceDirection::FD_South, this->snapshot_id, 5);
				//Or (x, y + 4)
				CheckAndAdd({ this->m_location.x, this->m_location.y + 2 + Config::get().Get_EXPANDED_CELL() * 2 }, FaceDirection::FD_South, this->snapshot_id);

				//Or (x + 1, y + 4)
				CheckAndAdd({ this->m_location.x + 1, this->m_location.y + 2 + Config::get().Get_EXPANDED_CELL() * 2 }, 
					FaceDirection::FD_South, this->snapshot_id, Config::get().Get_SCREENSHOT_COST());
				//Or (x - 1, y + 4)
				CheckAndAdd({ this->m_location.x - 1, this->m_location.y + 2 + Config::get().Get_EXPANDED_CELL() * 2 }, 
					FaceDirection::FD_South, this->snapshot_id, Config::get().Get_SCREENSHOT_COST());
			}
			else {
				//Or (x, y + 4)
				CheckAndAdd({ this->m_location.x, this->m_location.y + 2 + Config::get().Get_EXPANDED_CELL() * 2 }, 
					FaceDirection::FD_South, this->snapshot_id);
				//Or (x, y + 5)
				CheckAndAdd({ this->m_location.x, this->m_location.y + 3 + Config::get().Get_EXPANDED_CELL() * 2 }, 
					FaceDirection::FD_South, this->snapshot_id);
				//Or (x + 1, y + 4)
				CheckAndAdd({ this->m_location.x + 1, this->m_location.y + 2 + Config::get().Get_EXPANDED_CELL() * 2 }, 
					FaceDirection::FD_South, this->snapshot_id, Config::get().Get_SCREENSHOT_COST());
				//Or (x - 1, y + 4)
				CheckAndAdd({ this->m_location.x - 1, this->m_location.y + 2 + Config::get().Get_EXPANDED_CELL() * 2 }, 
					FaceDirection::FD_South, this->snapshot_id, Config::get().Get_SCREENSHOT_COST());
			}
			break;
		case FaceDirection::FD_South:
			if (!retrying) {
				//Or (x, y - 3)
				CheckAndAdd({ this->m_location.x, this->m_location.y - 1 - Config::get().Get_EXPANDED_CELL() * 2 }, 
					FaceDirection::FD_North, this->snapshot_id, 5);
				//Or (x, y - 4)
				CheckAndAdd({ this->m_location.x, this->m_location.y - 2 - Config::get().Get_EXPANDED_CELL() * 2 }, 
					FaceDirection::FD_North, this->snapshot_id);

				//Or (x + 1, y - 4)
				CheckAndAdd({ this->m_location.x + 1, this->m_location.y - 2 - Config::get().Get_EXPANDED_CELL() * 2 }, 
					FaceDirection::FD_North, this->snapshot_id, Config::get().Get_SCREENSHOT_COST());
				//Or (x - 1, y - 4)
				CheckAndAdd({ this->m_location.x - 1, this->m_location.y - 2 - Config::get().Get_EXPANDED_CELL() * 2 }, 
					FaceDirection::FD_North, this->snapshot_id, Config::get().Get_SCREENSHOT_COST());
			}
			else {
				//Or (x, y - 4)
				CheckAndAdd({ this->m_location.x, this->m_location.y - 2 - Config::get().Get_EXPANDED_CELL() * 2 }, 
					FaceDirection::FD_North, this->snapshot_id);
				//Or (x, y - 5)
				CheckAndAdd({ this->m_location.x, this->m_location.y - 3 - Config::get().Get_EXPANDED_CELL() * 2 }, 
					FaceDirection::FD_North, this->snapshot_id);
				//Or (x + 1, y - 4)
				CheckAndAdd({ this->m_location.x + 1, this->m_location.y - 2 - Config::get().Get_EXPANDED_CELL() * 2 }, 
					FaceDirection::FD_North, this->snapshot_id, Config::get().Get_SCREENSHOT_COST());
				//Or (x - 1, y - 4)
				CheckAndAdd({ this->m_location.x - 1, this->m_location.y - 2 - Config::get().Get_EXPANDED_CELL() * 2 }, 
					FaceDirection::FD_North, this->snapshot_id, Config::get().Get_SCREENSHOT_COST());
			}
			break;
		case FaceDirection::FD_East:
			if (!retrying) {
				//Or (x + 3, y)
				CheckAndAdd({ this->m_location.x + 1 + Config::get().Get_EXPANDED_CELL() * 2 , this->m_location.y }, 
					FaceDirection::FD_West, this->snapshot_id, 5);
				//Or (x + 4, y)
				CheckAndAdd({ this->m_location.x + 2 + Config::get().Get_EXPANDED_CELL() * 2 , this->m_location.y }, 
					FaceDirection::FD_West, this->snapshot_id);
				//Or (x + 4, y + 1)
				CheckAndAdd({ this->m_location.x + 2 + Config::get().Get_EXPANDED_CELL() * 2 , this->m_location.y + 1 }, 
					FaceDirection::FD_West, this->snapshot_id, Config::get().Get_SCREENSHOT_COST());
				//Or (x + 4, y - 1)
				CheckAndAdd({ this->m_location.x + 2 + Config::get().Get_EXPANDED_CELL() * 2 , this->m_location.y - 1 }, 
					FaceDirection::FD_West, this->snapshot_id, Config::get().Get_SCREENSHOT_COST());
			}
			else {
				//Or (x + 4, y)
				CheckAndAdd({ this->m_location.x + 2 + Config::get().Get_EXPANDED_CELL() * 2 , this->m_location.y }, 
					FaceDirection::FD_West, this->snapshot_id);
				//Or (x + 5, y)
				CheckAndAdd({ this->m_location.x + 3 + Config::get().Get_EXPANDED_CELL() * 2 , this->m_location.y }, 
					FaceDirection::FD_West, this->snapshot_id);
				//Or (x + 4, y + 1)
				CheckAndAdd({ this->m_location.x + 2 + Config::get().Get_EXPANDED_CELL() * 2 , this->m_location.y + 1 }, 
					FaceDirection::FD_West, this->snapshot_id, Config::get().Get_SCREENSHOT_COST());
				//Or (x + 4, y - 1)
				CheckAndAdd({ this->m_location.x + 2 + Config::get().Get_EXPANDED_CELL() * 2 , this->m_location.y - 1 }, 
					FaceDirection::FD_West, this->snapshot_id, Config::get().Get_SCREENSHOT_COST());
			}
			break;
		case FaceDirection::FD_West:
			if (!retrying) {
				//Or (x - 3, y)
				CheckAndAdd({ this->m_location.x - 1 - Config::get().Get_EXPANDED_CELL() * 2 , this->m_location.y }, 
					FaceDirection::FD_East, this->snapshot_id, 5);
				//Or (x - 4, y)
				CheckAndAdd({ this->m_location.x - 2 - Config::get().Get_EXPANDED_CELL() * 2 , this->m_location.y }, 
					FaceDirection::FD_East, this->snapshot_id);
				//Or (x - 4, y + 1)
				CheckAndAdd({ this->m_location.x - 2 - Config::get().Get_EXPANDED_CELL() * 2 , this->m_location.y + 1 }, 
					FaceDirection::FD_East, this->snapshot_id, Config::get().Get_SCREENSHOT_COST());
				//Or (x - 4, y - 1)
				CheckAndAdd({ this->m_location.x - 2 - Config::get().Get_EXPANDED_CELL() * 2 , this->m_location.y - 1 }, 
					FaceDirection::FD_East, this->snapshot_id, Config::get().Get_SCREENSHOT_COST());
			}
			else {
				//Or (x - 4, y)
				CheckAndAdd({ this->m_location.x - 2 - Config::get().Get_EXPANDED_CELL() * 2 , this->m_location.y }, 
					FaceDirection::FD_East, this->snapshot_id);
				//Or (x - 5, y)
				CheckAndAdd({ this->m_location.x - 3 - Config::get().Get_EXPANDED_CELL() * 2 , this->m_location.y }, 
					FaceDirection::FD_East, this->snapshot_id);
				//Or (x - 4, y + 1)
				CheckAndAdd({ this->m_location.x - 2 - Config::get().Get_EXPANDED_CELL() * 2 , this->m_location.y + 1 }, 
					FaceDirection::FD_East, this->snapshot_id, Config::get().Get_SCREENSHOT_COST());
				//Or (x - 4, y - 1)
				CheckAndAdd({ this->m_location.x - 2 - Config::get().Get_EXPANDED_CELL() * 2 , this->m_location.y - 1 }, 
					FaceDirection::FD_East, this->snapshot_id, Config::get().Get_SCREENSHOT_COST());
			}
			break;
		}
		return output;
	}

	/***********************************************************/

	FieldRobot::FieldRobot(const POINT& start_loc, FaceDirection fd) :
		FieldObject(start_loc, {3,3}, fd)
	{
	}

	void FieldRobot::ChangeDirection()
	{
		if (this->m_Fd >= FaceDirection::FD_West)
			this->m_Fd = FaceDirection::FD_North;
		else
			this->m_Fd = static_cast<FaceDirection>(static_cast<int>(this->m_Fd) + 1);
	}

	GridBoxType FieldRobot::GetGridBoxType(const POINT& loc)
	{
		/*if (this->m_location.x == loc.x && this->m_location.y == loc.y &&
			this->m_dimen.x == 1 && this->m_dimen.y == 1) {
			//1x1 block(should not happen)
			return GridBoxType::GBT_Robot;
		}*/
		if (!this->IsInRect(loc))
			return GridBoxType::GBT_Invalid;

		int dim_x = this->m_dimen.x / 2;
		int dim_y = this->m_dimen.y / 2;
		switch (this->m_Fd) {
		case FaceDirection::FD_None: return GridBoxType::GBT_Robot;
		case FaceDirection::FD_North: 
			if (loc.y == this->m_location.y + dim_y && 
				loc.x == this->m_location.x)
				return GridBoxType::GBT_Camera;
			break;
		case FaceDirection::FD_East: 
			if (loc.x == this->m_location.x + dim_x &&
				loc.y == this->m_location.y)
				return GridBoxType::GBT_Camera;
			break;
		case FaceDirection::FD_South:
			if (loc.y == this->m_location.y - dim_x &&
				loc.x == this->m_location.x)
				return GridBoxType::GBT_Camera;
			break;
		case FaceDirection::FD_West:
			if (loc.x == this->m_location.x - dim_x &&
				loc.y == this->m_location.y)
				return GridBoxType::GBT_Camera;
			break;
		}
		return GridBoxType::GBT_Robot;
	}
}