/*
 * PlanBase.h
 *
 *  Created on: Jun 17, 2014
 *      Author: Paul Panin
 */

#ifndef PLANBASE_H_
#define PLANBASE_H_
#define PB_DEBUG

using namespace std;

#include <queue>
#include <stdio.h>
#include <thread>
#include <AutoResetEvent.h>
#include <Timer.h>
#include <algorithm>
#include <math.h>
#include <mutex>
#include <memory>
#include <typeinfo>
#include "containers/BehaviourEngineInfo.h"
#include "engine/IAlicaClock.h"
#include "engine/RunningPlan.h"

namespace alica
{
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
	class AlicaEngine;

	/**
	 * A PlanBase holds the internal representation of the plan graph and issues all operations on it.
	 * It is the most central object within the ALICA Engine.
	 */
	class PlanBase
	{
	public:
		PlanBase(AlicaEngine* ae, Plan* masterplan);
		~PlanBase();
		condition_variable* getStepModeCV();
		const shared_ptr<RunningPlan> getRootNode() const;
		void setRootNode(shared_ptr<RunningPlan> rootNode);
		void setRuleBook(RuleBook* ruleBook);
		const ulong getloopInterval() const;
		void setLoopInterval(ulong loopInterval);
		void stop();
		void start();
		void addFastPathEvent(shared_ptr<RunningPlan> p);
		shared_ptr<RunningPlan> getDeepestNode();
		shared_ptr<RunningPlan> getRootNode();
		Plan* getMasterPlan();

	private:
		/**
		 * List of RunningPlans scheduled for out-of-loop evaluation.
		 */
		queue<shared_ptr<RunningPlan>> fpEvents;
//		supplementary::AutoResetEvent* signal;
//		supplementary::AutoResetEvent* loopGuard;
		condition_variable* timerModeCV;
		condition_variable* stepModeCV;
		supplementary::Timer* loopTimer;
		void checkPlanBase(shared_ptr<RunningPlan> r);

	protected:
		Plan* masterPlan;
		shared_ptr<RunningPlan> rootNode;
		shared_ptr<RunningPlan> deepestNode;
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
