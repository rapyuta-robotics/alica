#ifndef PLANTREEINFO_H_
#define PLANTREEINFO_H_

#include <vector>

using namespace std;

namespace alica
{
	struct PlanTreeInfo
	{
		int senderID;
		list<long> stateIDs;
		list<long> succeededEPs;
	};
}



#endif /* BEHAVIOURENGINEINFO_H_ */
