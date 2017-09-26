#pragma once

#include <vector>
#include <tuple>

using namespace std;

namespace alica
{
	typedef tuple<const alica::IRobotID*, list<long>, list<long>> stdPlanTreeInfo;
	struct PlanTreeInfo
	{
		PlanTreeInfo() : senderID(nullptr)
		{
		}
		const alica::IRobotID* senderID;
		list<long> stateIDs;
		list<long> succeededEPs;

		PlanTreeInfo(stdPlanTreeInfo &s)
		{
			this->senderID = get<0>(s);
			this->stateIDs = get<1>(s);
			this->succeededEPs = get<2>(s);
		}

		stdPlanTreeInfo toStandard()
		{
			return move(make_tuple(senderID, stateIDs, succeededEPs));
		}
	};
} /* namespace alica*/
