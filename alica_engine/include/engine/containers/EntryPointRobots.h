#pragma once

#include "engine/IRobotID.h"

#include <vector>
#include <tuple>

namespace alica
{
	using std::vector;
	using std::tuple;
	using std::get;

	typedef tuple<long, vector<const alica::IRobotID*>> stdEntryPointRobot;
	struct EntryPointRobots
	{
		EntryPointRobots() : entrypoint(0)
		{
		}

		long entrypoint;
		vector<const alica::IRobotID*> robots;

		EntryPointRobots(stdEntryPointRobot& s)
		{
			entrypoint = get<0>(s);
			robots = get<1>(s);
		}

		stdEntryPointRobot toStandard()
		{
			return move(make_tuple(entrypoint, robots));
		}
	};
}
