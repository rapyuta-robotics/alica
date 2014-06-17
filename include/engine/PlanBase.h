/*
 * PlanBase.h
 *
 *  Created on: Jun 17, 2014
 *      Author: snook
 */

#ifndef PLANBASE_H_
#define PLANBASE_H_

using namespace std;

#include <thread>
#include <queue>

namespace alica
{
	class RunningPlan;
	class Plan;
	class RuleBook;
	class AlicaEngine;
	class ITeamObserver;
	class IRoleAssignment;
	class Logger;
	class AuthorityManager;
	class ISyncModul;

	class PlanBase
	{
	public:
		PlanBase(Plan* masterplan);
		virtual ~PlanBase();

	private:
		queue<RunningPlan> fpEvents;

	protected:
		Plan* masterPlan;
		RunningPlan* rootNode;
		RunningPlan* deepestNode;
		AlicaEngine* ae;
		int treeDepth;
		RuleBook* ruleBook;
		ITeamObserver* teamObserver;
		IRoleAssignment* ra;
		ISyncModul* syncModel;
		AuthorityManager* authModul;

		ulong loopTime;
		ulong lastSendTime;
		ulong minSendInterval;
		ulong maxSendInterval;
		ulong loopInterval;

		bool ruuning;
		bool sendStatusMessage;
		bool sendStatusInterval;

		thread mainThread;
		Logger* log;

	};

} /* namespace Alica */
#endif /* PLANBASE_H_ */
