/*
 * EntryPointRobots.h
 *
 *  Created on: Jul 7, 2014
 *      Author: Paul Panin
 */

#ifndef ENTRYPOINTROBOTS_H_
#define ENTRYPOINTROBOTS_H_

#include <vector>
#include <tuple>

using namespace std;

namespace alica
{
	typedef tuple<int, vector<int>> stdEntryPointRobot;
	struct EntryPointRobots
	{
		EntryPointRobots()
		{
		}

		int entrypoint;
		vector<int> robots;

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

#endif /* ENTRYPOINTROBOTS_H_ */
