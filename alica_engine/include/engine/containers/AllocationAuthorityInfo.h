#pragma once

#include "engine/IRobotID.h"
#include "EntryPointRobots.h"

#include <vector>
#include <tuple>

namespace alica
{
	using std::tuple;
	using std::vector;

	typedef tuple<const alica::IRobotID*, long, long, long, const alica::IRobotID*, vector<stdEntryPointRobot>> stdAllocationAuthorityInfo;
	struct AllocationAuthorityInfo
	{
		AllocationAuthorityInfo()
		{
		}

		const alica::IRobotID* senderID;
		long planId;
		long parentState;
		long planType;
		const alica::IRobotID* authority;
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
} /* namespace alica */
