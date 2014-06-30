/*
 * PlanBase.h
 *
 *  Created on: Jun 17, 2014
 *      Author: Paul Panin
 */

#ifndef PLANBASE_H_
#define PLANBASE_H_

using namespace std;

#include <queue>
#include <stdio.h>
#include <thread>
#include <AutoResetEvent.h>
#include <Timer.h>
#include <algorithm>
#include <math.h>
#include "containers/BehaviourEngineInfo.h"

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
	class IAlicaCommunication;

	class PlanBase
	{
	public:
		PlanBase(Plan* masterplan);
		~PlanBase();
		supplementary::AutoResetEvent* getSignal();
		const RunningPlan* getRootNode() const;
		void setRootNode(RunningPlan* rootNode);

		void start();

	private:
		queue<RunningPlan> fpEvents;
		supplementary::AutoResetEvent* signal;
		supplementary::AutoResetEvent* loopGuard;
		supplementary::Timer* loopTimer;

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
		IAlicaCommunication* statusPublisher;

		ulong loopTime;
		ulong lastSendTime;
		ulong minSendInterval;
		ulong maxSendInterval;
		ulong loopInterval;

		bool ruuning;
		bool sendStatusMessages;
		bool sendStatusInterval;

		thread* mainThread;
		Logger* log;

		BehaviourEngineInfo statusMessage;

		void run();

	};

} /* namespace Alica */
#endif /* PLANBASE_H_ */
