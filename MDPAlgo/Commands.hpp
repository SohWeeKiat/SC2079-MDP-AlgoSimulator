#pragma once
#include <string>
#include <vector>
#include "FieldObjects.hpp"

namespace MDP {
	enum CommandType {
		CT_ForwardLeft,
		CT_ForwardRight,
		CT_BackwardLeft,
		CT_BackwardRight,
		CT_Forward,
		CT_Backward,
		CT_Snap,
		CT_Finish,
	};

	class Command {

	public:
		Command(CommandType type, int distance = 0, const ObjectState& state = ObjectState());
		//for snap command
		Command(CommandType type, int distance, const ObjectState& state,  const char direction);

		CommandType GetType() const;
		int GetDistance() const;
		ObjectState GetState() const;
		void SetDistance(int distance, const ObjectState& next_state);
		std::string ToString();
	private:
		CommandType type;
		int distance;
		ObjectState state;
		char snap_direction;
	};

	std::vector<Command> command_generator(const std::vector<ObjectState>& states,
		const std::vector<SFieldObject>& obstacles);
}