/*
 * AllocationAuthorityInfo.h
 *
 *  Created on: Jul 7, 2014
 *      Author: Paul Panin
 */

#ifndef ALLOCATIONAUTHORITYINFO_H_
#define ALLOCATIONAUTHORITYINFO_H_

#include "engine/IRobotID.h"
#include "EntryPointRobots.h"

#include <vector>
#include <tuple>

using namespace std;

namespace alica
{
	typedef tuple<alica::IRobotID, long, long, long, alica::IRobotID, vector<stdEntryPointRobot>> stdAllocationAuthorityInfo;
	struct AllocationAuthorityInfo
	{
		AllocationAuthorityInfo()
		{
		}

		alica::IRobotID senderID;
		long planId;
		long parentState;
		long planType;
		alica::IRobotID authority;
		vector<EntryPointRobots> entryPointRobots;

		AllocationAuthorityInfo(stdAllocationAuthorityInfo &s)
		{
			this->senderID = get<0>(s);
			this->planId = get<1>(s);
			this->parentState = get<2>(s);
			this->planType = get<3>(s);
			this->authority = get<4>(s);
			vector<stdEntryPointRobot>& tmp = get<5>(s);
			for (stdEntryPointRobot& e : tmp)
			{
				this->entryPointRobots.push_back(EntryPointRobots(e));
			}
		}

		stdAllocationAuthorityInfo toStandard()
		{
			vector<stdEntryPointRobot> r;
			for (EntryPointRobots& e : entryPointRobots)
			{
				r.push_back(move(e.toStandard()));
			}
			return move(make_tuple(senderID, planId, parentState, planType, authority, move(r)));
		}
	};
}

#endif /* ALLOCATIONAUTHORITYINFO_H_ */
