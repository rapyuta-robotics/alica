#pragma once

#include "container/CNPosition.h"

namespace msl
{
namespace robot
{
	class IntRobotID;
}
}

namespace geometry
{

	class CNRobot : public CNPosition
	{
	public:
		CNRobot();
		virtual ~CNRobot();
		double radius;
		double velocityX;
		double velocityY;
		const msl::robot::IntRobotID* id;
		shared_ptr<vector<int>> opposer;
		shared_ptr<vector<int>> supporter;
		double certainty;
		double rotation;
		string toString();
	};
}
