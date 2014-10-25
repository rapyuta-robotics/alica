/*
 * BehaviourEngineInfo.h
 *
 *  Created on: Jun 30, 2014
 *      Author: Paul Panin
 */

#ifndef BEHAVIOURENGINEINFO_H_
#define BEHAVIOURENGINEINFO_H_

#include <vector>

using namespace std;

namespace alica
{
	typedef tuple<int, string, string, string, string, string, vector<int>> stdBehaviourEngineInfo;
	struct BehaviourEngineInfo
	{
		BehaviourEngineInfo()
		{
		}
		int senderID;
		string masterPlan;
		string currentPlan;
		string currentState;
		string currentRole;
		string currentTask;
		vector<int> robotIDsWithMe;

		BehaviourEngineInfo(stdBehaviourEngineInfo &s)
		{
			senderID = get<0>(s);
			masterPlan = move(get<1>(s));
			currentPlan = move(get<2>(s));
			currentState = move(get<3>(s));
			currentRole = move(get<4>(s));
			currentTask = move(get<5>(s));
			robotIDsWithMe = move(get<6>(s));
		}

		stdBehaviourEngineInfo toStandard()
		{
			return move(
					make_tuple(senderID, masterPlan, currentPlan, currentState, currentRole, currentTask,
								robotIDsWithMe));
		}
	};
}

#endif /* BEHAVIOURENGINEINFO_H_ */
