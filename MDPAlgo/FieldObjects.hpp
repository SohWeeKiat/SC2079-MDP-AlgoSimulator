#pragma once
#include <Windows.h>
#include <memory>
#include <vector>

namespace MDP {
	enum FaceDirection {
		FD_None,
		FD_North,
		FD_East,
		FD_South,
		FD_West,
	};
	std::ostream& operator<<(std::ostream& os, const FaceDirection& obj);

	enum GridBoxType {
		GBT_Invalid,
		GBT_None,
		GBT_North,
		GBT_East,
		GBT_South,
		GBT_West,

		GBT_Robot,
		GBT_Camera,
	};

	struct ObjectState {
		POINT m_location;
		FaceDirection m_Fd;
		int penalty;
		int snapshot_id;

		ObjectState() = default;

		ObjectState(const POINT& start_loc, FaceDirection fd, int penalty = 0, int snapshot_id = -1) :
			m_location(start_loc),  m_Fd(fd), snapshot_id(snapshot_id), penalty(penalty)
		{
		}
		
		ObjectState(const ObjectState& rhs) :
			m_location(rhs.m_location), m_Fd(rhs.m_Fd), snapshot_id(rhs.snapshot_id),
			penalty(rhs.penalty)
		{

		}

		bool operator==(const ObjectState& rhs) const
		{
			return this->m_location.x == rhs.m_location.x &&
				this->m_location.y == rhs.m_location.y &&
				this->m_Fd == rhs.m_Fd;
		}

		bool operator!=(const ObjectState& rhs) const
		{
			return this->m_location.x != rhs.m_location.x ||
				this->m_location.y != rhs.m_location.y ||
				this->m_Fd != rhs.m_Fd;
		}

		bool operator<(const ObjectState& rhs) const
		{
			return this->m_location.x < rhs.m_location.x ||
				this->m_location.y < rhs.m_location.y ||
				this->m_Fd < rhs.m_Fd;
		}
	};
	std::ostream& operator<<(std::ostream& os, ObjectState& obj);
	std::ostream& operator<<(std::ostream& os, std::vector<ObjectState>& obj);

	struct FieldStartEnd {
		ObjectState Start;
		ObjectState End;

		bool operator==(const FieldStartEnd& rhs) const
		{
			return this->Start == rhs.Start &&
				this->End == rhs.End;
		}

		bool operator<(const FieldStartEnd& rhs) const
		{
			return this->Start < rhs.Start ||
				this->End < rhs.End;
		}
	};

	class FieldObject : protected ObjectState {

	public:
		FieldObject(const POINT& start_loc, const POINT& dimen = POINT{ 1,1 },
			FaceDirection fd = FaceDirection::FD_North);

		void Update(const POINT& loc, FaceDirection fd);
		void UpdateSnapshotID(int ID);
		bool IsInRect(const POINT& loc);
		virtual void ChangeDirection();
		std::vector<POINT> GetRectPoints();
		FaceDirection GetDirection() const;
		POINT GetLoc() const;
		int GetSnapshotID() const;
		ObjectState& GetState();

		virtual std::vector<ObjectState> GetViewState(bool retrying);
		virtual GridBoxType GetGridBoxType(const POINT& loc) = 0;
	protected:
		POINT m_dimen;
	};
	typedef std::shared_ptr<FieldObject> SFieldObject;

	class FieldBlock : public FieldObject {

	public:
		FieldBlock(const POINT& start_loc, FaceDirection fd = FaceDirection::FD_North, 
			int obstacle_id = 1);
		virtual GridBoxType GetGridBoxType(const POINT& loc) override;
		virtual std::vector<ObjectState> GetViewState(bool retrying) override;
	};

	class FieldRobot : public FieldObject {

	public:
		FieldRobot(const POINT& start_loc = {1,1}, FaceDirection fd = FaceDirection::FD_North);
		virtual void ChangeDirection() override;
		virtual GridBoxType GetGridBoxType(const POINT& loc) override;

	private:
		std::vector<ObjectState> m_states;
	};
}

template <>
struct std::hash<MDP::ObjectState>
{
	std::size_t operator()(const MDP::ObjectState& k) const
	{
		using std::hash;

		std::size_t res = 17;
		res = res * 31 + hash<int>()(k.m_location.x);
		res = res * 31 + hash<int>()(k.m_location.y);
		res = res * 31 + hash<int>()(k.m_Fd);
		return res;
	}
};

template <>
struct std::hash<MDP::FieldStartEnd>
{
	std::size_t operator()(const MDP::FieldStartEnd& k) const
	{
		using std::hash;

		std::size_t res = 17;
		res = res * 31 + hash< MDP::ObjectState>()(k.Start);
		res = res * 31 + hash< MDP::ObjectState>()(k.End);
		return res;
	}
};
