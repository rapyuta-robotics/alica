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
#include <mutex>
#include <typeinfo>
#include "containers/BehaviourEngineInfo.h"
#include "engine/IAlicaClock.h"

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
	class Task;
	class State;
	class EntryPoint;
	class IAlicaClock;
	class Assignment;
	class StateCollection;

	class PlanBase
	{
	public:
		PlanBase(Plan* masterplan);
		~PlanBase();
		condition_variable* getStepModeCV();
		const RunningPlan* getRootNode() const;
		void setRootNode(RunningPlan* rootNode);
		void setRuleBook(RuleBook* ruleBook);
		const ulong getloopInterval() const;
		void setLoopInterval(ulong loopInterval);
		void stop();
		void start();
		void addFastPathEvent(RunningPlan* p);

	private:
		queue<RunningPlan*> fpEvents;
//		supplementary::AutoResetEvent* signal;
//		supplementary::AutoResetEvent* loopGuard;
		condition_variable* timerModeCV;
		condition_variable* stepModeCV;
		supplementary::Timer* loopTimer;
		void checkPlanBase(RunningPlan* r);

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
		IAlicaClock* alicaClock;

		alicaTime loopTime;
		alicaTime lastSendTime;
		alicaTime minSendInterval;
		alicaTime maxSendInterval;
		alicaTime loopInterval;
		alicaTime lastSentStatusTime;
		alicaTime sendStatusInterval;

		bool running;
		bool sendStatusMessages;

		thread* mainThread;
		Logger* log;

		BehaviourEngineInfo* statusMessage;
		mutex lomutex;
		mutex stepMutex;
		mutex timerMutex;
		unique_lock<mutex> lckTimer;
		void run();


	};

} /* namespace Alica */
#endif /* PLANBASE_H_ */
