#include "Commands.hpp"
#include <map>
#include "Config.hpp"

namespace MDP {
	const int BaseSpeed = 0;

	template<typename T>
	std::string ZeroPad(const T& value, int n_zero = 2)
	{
		std::string output = std::to_string(value);
		return std::string(n_zero - min(n_zero, output.length()), '0') + output;
	}

	Command::Command(CommandType type, int distance, const ObjectState& state) :
		type(type), distance(distance), state(state), snap_direction('\0')
	{

	}

	//for snap command
	Command::Command(CommandType type, int distance, const ObjectState& state, const char direction) :
		type(type), distance(distance), state(state), snap_direction(direction)
	{

	}

	CommandType Command::GetType() const
	{
		return this->type;
	}

	int Command::GetDistance() const
	{
		return this->distance;
	}

	ObjectState Command::GetState() const
	{
		return this->state;
	}

	void Command::SetDistance(int distance, const ObjectState& next_state)
	{
		this->distance = distance;
		this->state = next_state;
	}

	std::string Command::ToString()
	{
		std::string output;
		switch (this->type){
		case CommandType::CT_ForwardLeft: output += "FL"; break;
		case CommandType::CT_ForwardRight: output += "FR"; break;
		case CommandType::CT_BackwardLeft: output += "BL"; break;
		case CommandType::CT_BackwardRight: output += "BR"; break;
		case CommandType::CT_Forward: output += "FW"; break;
		case CommandType::CT_Backward: output += "BW"; break;
		case CommandType::CT_Snap: output += "SNAP"; break;
		case CommandType::CT_Finish: output += "FIN"; break;
		}
		if (this->type >= CommandType::CT_ForwardLeft && this->type <= CommandType::CT_BackwardRight)
			output += ZeroPad(this->distance);
		else if (this->type >= CommandType::CT_Forward && this->type <= CommandType::CT_Backward)
			output += ZeroPad(this->distance);
		else if (this->type == CommandType::CT_Snap) {
			output += std::to_string(this->distance);
			if (this->snap_direction != '\0') 
				output += std::string("_") + std::string(1, this->snap_direction);
		}
		return output;
	}

	std::vector<Command> compress_commands(const std::vector<Command>& commands)
	{
		std::vector<Command> result;
		if (!commands.empty()) result.push_back(commands[0]);
		for (std::size_t i = 1; i < commands.size(); i++)
		{
			auto& last_entry = result[result.size() - 1];
			if (commands[i].GetType() == CommandType::CT_Backward &&
				commands[i].GetType() == last_entry.GetType()) {
				int steps = last_entry.GetDistance();
					if (steps < 90 || !Config::get().Is_LimitMax90()) {
						last_entry.SetDistance(last_entry.GetDistance() + 10, commands[i].GetState());
						continue;
					}
			}else if (commands[i].GetType() == CommandType::CT_Forward &&
				commands[i].GetType() == last_entry.GetType()) {
				int steps = last_entry.GetDistance();
				if (steps < 90 || !Config::get().Is_LimitMax90()) {
					last_entry.SetDistance(last_entry.GetDistance() + 10, commands[i].GetState());
					continue;
				}
			}
			result.push_back(commands[i]);
		}
		return result;
	}

	void handle_snapshot_requirement(std::vector<Command>& commands,const SFieldObject& obj, const ObjectState& state)
	{
		if (obj->GetDirection() == FaceDirection::FD_West && state.m_Fd == FaceDirection::FD_East) {
			if (obj->GetLoc().y > state.m_location.y) 
				commands.push_back(Command(CommandType::CT_Snap, state.snapshot_id, state, 'L'));
			else if (obj->GetLoc().y == state.m_location.y) 
				commands.push_back(Command(CommandType::CT_Snap, state.snapshot_id, state, 'C'));
			else if (obj->GetLoc().y < state.m_location.y) 
				commands.push_back(Command(CommandType::CT_Snap, state.snapshot_id, state, 'R'));
			else
				commands.push_back(Command(CommandType::CT_Snap, state.snapshot_id, state, '\0'));
		}else if (obj->GetDirection() == FaceDirection::FD_East && state.m_Fd == FaceDirection::FD_West) {
			if (obj->GetLoc().y > state.m_location.y)
				commands.push_back(Command(CommandType::CT_Snap, state.snapshot_id, state, 'R'));
			else if (obj->GetLoc().y == state.m_location.y)
				commands.push_back(Command(CommandType::CT_Snap, state.snapshot_id, state, 'C'));
			else if (obj->GetLoc().y < state.m_location.y)
				commands.push_back(Command(CommandType::CT_Snap, state.snapshot_id, state, 'L'));
			else
				commands.push_back(Command(CommandType::CT_Snap, state.snapshot_id, state, '\0'));
		}
		else if (obj->GetDirection() == FaceDirection::FD_North && state.m_Fd == FaceDirection::FD_South) {
			if (obj->GetLoc().x > state.m_location.x)
				commands.push_back(Command(CommandType::CT_Snap, state.snapshot_id, state, 'L'));
			else if (obj->GetLoc().x == state.m_location.x)
				commands.push_back(Command(CommandType::CT_Snap, state.snapshot_id, state, 'C'));
			else if (obj->GetLoc().x < state.m_location.x)
				commands.push_back(Command(CommandType::CT_Snap, state.snapshot_id, state, 'R'));
			else
				commands.push_back(Command(CommandType::CT_Snap, state.snapshot_id, state, '\0'));
		}
		else if (obj->GetDirection() == FaceDirection::FD_South && state.m_Fd == FaceDirection::FD_North) {
			if (obj->GetLoc().x > state.m_location.x)
				commands.push_back(Command(CommandType::CT_Snap, state.snapshot_id, state, 'R'));
			else if (obj->GetLoc().x == state.m_location.x)
				commands.push_back(Command(CommandType::CT_Snap, state.snapshot_id, state, 'C'));
			else if (obj->GetLoc().x < state.m_location.x)
				commands.push_back(Command(CommandType::CT_Snap, state.snapshot_id, state, 'L'));
			else
				commands.push_back(Command(CommandType::CT_Snap, state.snapshot_id, state, '\0'));
		}
	}

	std::vector<Command> command_generator(const std::vector<ObjectState>& states,
		const std::vector<SFieldObject>& obstacles)
	{
		std::vector<Command> commands;
		std::map<int, SFieldObject> obstacle_map;
		for (auto& o : obstacles) obstacle_map[o->GetSnapshotID()] = o;

		int OutsideCommandValue = Config::get().Is_OutsideCommand() ? 30 : BaseSpeed;

		for (std::size_t i = 1; i < states.size();i++)
		{
			auto& c_state = states[i];
			auto& p_state = states[i - 1];

			//If previous state and current state are the same direction,
			if (c_state.m_Fd == p_state.m_Fd) {
				if ((c_state.m_location.x > p_state.m_location.x && c_state.m_Fd == FaceDirection::FD_East) || //going horizonal right
					(c_state.m_location.y > p_state.m_location.y && c_state.m_Fd == FaceDirection::FD_North)) {//going up
					commands.push_back(Command(CommandType::CT_Forward, 10, c_state));
					
				}
				//Forward - Must be(west facing AND x value decreased) OR(south facing AND y value decreased)
				else if ((c_state.m_location.x < p_state.m_location.x && c_state.m_Fd == FaceDirection::FD_West) || //going horizonal left
					(c_state.m_location.y < p_state.m_location.y && c_state.m_Fd == FaceDirection::FD_South)){//going down
					commands.push_back(Command(CommandType::CT_Forward, 10, c_state));
						
				}
				//Backward - All other cases where the previous and current state is the same direction
				else {
					commands.push_back(Command(CommandType::CT_Backward, 10, c_state));
				}
				//If any of these states has a valid screenshot ID, then add a SNAP command as well to take a picture
				if (c_state.snapshot_id != -1)
					handle_snapshot_requirement(commands, obstacle_map[c_state.snapshot_id], c_state);
				continue;
			}
			/*
			If previous state and current state are not the same direction, it means that there will be a turn command involved
				Assume there are 4 turning command : FR, FL, BL, BR(the turn command will turn the robot 90 degrees)
				FR00 | FR30: Forward Right;
			FL00 | FL30: Forward Left;
			BR00 | BR30: Backward Right;
			BL00 | BL30: Backward Left;
			*/
			switch (p_state.m_Fd)
			{
			case FaceDirection::FD_North:
				if (c_state.m_Fd == FaceDirection::FD_East) {
					if (c_state.m_location.y > p_state.m_location.y)
						commands.push_back(Command(CommandType::CT_ForwardRight, OutsideCommandValue, c_state));
					else
						commands.push_back(Command(CommandType::CT_BackwardLeft, OutsideCommandValue, c_state));
				}else if (c_state.m_Fd == FaceDirection::FD_West) {
					if (c_state.m_location.y > p_state.m_location.y)
						commands.push_back(Command(CommandType::CT_ForwardLeft, OutsideCommandValue, c_state));
					else
						commands.push_back(Command(CommandType::CT_BackwardRight, OutsideCommandValue, c_state));
				}
				else
					throw "Invalid turning direction";
				break;
			case FaceDirection::FD_East:
				if (c_state.m_Fd == FaceDirection::FD_North) {
					if (c_state.m_location.y > p_state.m_location.y)
						commands.push_back(Command(CommandType::CT_ForwardLeft, OutsideCommandValue, c_state));
					else
						commands.push_back(Command(CommandType::CT_BackwardRight, OutsideCommandValue, c_state));
				}
				else if (c_state.m_Fd == FaceDirection::FD_South) {
					if (c_state.m_location.y > p_state.m_location.y)
						commands.push_back(Command(CommandType::CT_BackwardLeft, OutsideCommandValue, c_state));
					else
						commands.push_back(Command(CommandType::CT_ForwardRight, OutsideCommandValue, c_state));
				}
				else
					throw "Invalid turning direction";
				break;
			case FaceDirection::FD_South:
				if (c_state.m_Fd == FaceDirection::FD_East) {
					if (c_state.m_location.y > p_state.m_location.y)
						commands.push_back(Command(CommandType::CT_BackwardRight, OutsideCommandValue, c_state));
					else
						commands.push_back(Command(CommandType::CT_ForwardLeft, OutsideCommandValue, c_state));
				}
				else if (c_state.m_Fd == FaceDirection::FD_West) {
					if (c_state.m_location.y > p_state.m_location.y)
						commands.push_back(Command(CommandType::CT_BackwardLeft, OutsideCommandValue, c_state));
					else
						commands.push_back(Command(CommandType::CT_ForwardRight, OutsideCommandValue, c_state));
				}
				else
					throw "Invalid turning direction";
				break;
			case FaceDirection::FD_West:
				if (c_state.m_Fd == FaceDirection::FD_North) {
					if (c_state.m_location.y > p_state.m_location.y)
						commands.push_back(Command(CommandType::CT_ForwardRight, OutsideCommandValue, c_state));
					else
						commands.push_back(Command(CommandType::CT_BackwardLeft, OutsideCommandValue, c_state));
				}
				else if (c_state.m_Fd == FaceDirection::FD_South) {
					if (c_state.m_location.y > p_state.m_location.y)
						commands.push_back(Command(CommandType::CT_BackwardRight, OutsideCommandValue, c_state));
					else
						commands.push_back(Command(CommandType::CT_ForwardLeft, OutsideCommandValue, c_state));
				}
				else
					throw "Invalid turning direction";
				break;
			default:
				throw "Invalid position";
			}

			if (c_state.snapshot_id != -1)
				handle_snapshot_requirement(commands, obstacle_map[c_state.snapshot_id], c_state);
		}
		commands.push_back(Command(CommandType::CT_Finish));
		return compress_commands(commands);
		/*std::vector<std::string> output;
		for (auto& c : compressed_commands)
			output.push_back(c.ToString());

		return output;*/
	}
}