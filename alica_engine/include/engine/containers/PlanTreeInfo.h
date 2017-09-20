#ifndef PLANTREEINFO_H_
#define PLANTREEINFO_H_

#include <vector>
#include <tuple>

using namespace std;

namespace alica
{
	typedef tuple<alica::IRobotID, list<long>, list<long>> stdPlanTreeInfo;
	struct PlanTreeInfo
	{
		PlanTreeInfo()
		{
		}
		alica::IRobotID senderID;
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
}

#endif /* BEHAVIOURENGINEINFO_H_ */
