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
	struct BehaviourEngineInfo
	{
		int senderID;
		string masterPlan;
		string currentPlan;
		string currentState;
		string currentRole;
		string currentTask;
		vector<int> robotIDsWithMe;
	};
}



#endif /* BEHAVIOURENGINEINFO_H_ */
