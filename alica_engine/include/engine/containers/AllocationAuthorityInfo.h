/*
 * AllocationAuthorityInfo.h
 *
 *  Created on: Jul 7, 2014
 *      Author: Paul Panin
 */

#ifndef ALLOCATIONAUTHORITYINFO_H_
#define ALLOCATIONAUTHORITYINFO_H_

#include <vector>
#include <tuple>
#include "EntryPointRobots.h"

using namespace std;

namespace alica
{
	typedef tuple<int, long, long, long, int, vector<stdEntryPointRobot>> stdAllocationAuthorityInfo;
	struct AllocationAuthorityInfo
	{
		AllocationAuthorityInfo()
		{
		}

		int senderID;
		long planId;
		long parentState;
		long planType;
		int authority;
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
