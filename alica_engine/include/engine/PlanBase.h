#pragma once

#include <engine/containers/AlicaEngineInfo.h>
#include <queue>
#include <stdio.h>
#include <thread>
#include <condition_variable>
#include <algorithm>
#include <math.h>
#include <mutex>
#include <memory>
#include <typeinfo>
#include "engine/IAlicaClock.h"
#include "engine/RunningPlan.h"

using namespace std;
namespace alica
{
	class Plan;
	class RuleBook;
	class AlicaEngine;
	class TeamObserver;
	class IRoleAssignment;
	class Logger;
	class AuthorityManager;
	class SyncModul;
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
		condition_variable* stepModeCV;
//		void checkPlanBase(shared_ptr<RunningPlan> r);

	protected:
		Plan* masterPlan;
		shared_ptr<RunningPlan> rootNode;
		shared_ptr<RunningPlan> deepestNode;
		AlicaEngine* ae;
		int treeDepth;
		RuleBook* ruleBook;
		TeamObserver* teamObserver;
		IRoleAssignment* ra;
		SyncModul* syncModel;
		AuthorityManager* authModul;
		IAlicaCommunication* statusPublisher;
		IAlicaClock* alicaClock;

		AlicaTime loopTime;
		AlicaTime lastSendTime;
		AlicaTime minSendInterval;
		AlicaTime maxSendInterval;
		AlicaTime loopInterval;
		AlicaTime lastSentStatusTime;
		AlicaTime sendStatusInterval;

		bool running;
		bool sendStatusMessages;

		thread* mainThread;
		Logger* log;

		AlicaEngineInfo* statusMessage;
		mutex lomutex;
		mutex stepMutex;
		void run();


	};

} /* namespace Alica */
