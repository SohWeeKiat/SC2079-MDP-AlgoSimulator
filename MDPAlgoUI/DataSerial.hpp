#pragma once
#include "../MDPAlgo/FieldObjects.hpp"
#include <QDataStream>

QDataStream& operator<<(QDataStream& out, const MDP::SFieldObject& item)
{
	out << item->GetSnapshotID()
		<< (int)item->GetLoc().x
		<< (int)item->GetLoc().y
		<< item->GetDirection();
	return out;
}