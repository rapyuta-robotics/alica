/*
 * AlicaEngine.h
 *
 *  Created on: Mar 3, 2014
 *      Author: Stephan Opfer
 */

#ifndef ALICAENGINE_H_
#define ALICAENGINE_H_

using namespace std;

#include <string>
#include <SystemConfig.h>
#include <list>
#include <map>

namespace alica
{
	class PlanRepository;
	class Plan;
	class IPlanParser;
	class IBehaviourPool;
	class Logger;
	class RoleSet;
	class ITeamObserver;
	class IBehaviourCreator;
	class ISyncModul;
	class AuthorityManager;
	class IRoleAssignment;
	class IPlanSelector;
	class IAlicaCommunication;
	class IEngineModule;
	class IPlanner;
	class IAlicaClock;
	class PlanBase;
	class IConditionCreator;
	class IConstraintCreator;
	class IUtilityCreator;
	class ExpressionHandler;
	class PartialAssignmentPool;
	class IConstraintSolver;
	class IVariableSyncModule;

	class AlicaEngine
	{
	public:
		AlicaEngine();
//		static AlicaEngine* getInstance();
		bool init(IBehaviourCreator* bc, IConditionCreator* cc,  IUtilityCreator* uc, IConstraintCreator* crc, string roleSetName, string masterPlanName,
													string roleSetDir, bool stepEngine);
		void shutdown();
		void start();bool getStepEngine();
		void abort(string msg);
		template<typename T> void abort(string msg, const T tail);
		PlanRepository* getPlanRepository();
		IBehaviourPool* getBehaviourPool();
		string getRobotName() const;
		Logger* getLog();
		void setLog(Logger* log);
		ITeamObserver* getTeamObserver();
		void setTeamObserver(ITeamObserver* teamObserver);

		void setSyncModul(ISyncModul* syncModul);
		ISyncModul* getSyncModul();
		AuthorityManager* getAuth();
		void setAuth(AuthorityManager* auth);
		IRoleAssignment* getRoleAssignment();
		void setRoleAssignment(IRoleAssignment* roleAssignment);
		IPlanParser* getPlanParser();bool isTerminating() const;
		void setTerminating(bool terminating);
		void setStepCalled(bool stepCalled);
		bool getStepCalled() const;
		bool isMaySendMessages() const;
		void setMaySendMessages(bool maySendMessages);
		RoleSet* getRoleSet();
		IAlicaCommunication* getCommunicator();
		void setCommunicator(IAlicaCommunication * communicator);
		IPlanSelector* getPlanSelector();
		IPlanner* getPlanner();
		IAlicaClock* getIAlicaClock();
		void setIAlicaClock(IAlicaClock* clock);
		void doStep();
		void iterationComplete();
		PartialAssignmentPool* getPartialAssignmentPool();
		void stepNotify();
		PlanBase* getPlanBase();
		void addSolver(int identifier, IConstraintSolver* solver);
		IConstraintSolver* getSolver(int identifier);
		IVariableSyncModule* getResultStore();
		void setResultStore(IVariableSyncModule* resultStore);

		~AlicaEngine();

		/**
		 * Switch the engine between normal operation and silent mode, in which no messages other than debugging information are sent out.
		 * This is useful for a robot on hot standby.
		 */
		bool maySendMessages;

	protected:
		supplementary::SystemConfig* sc;
		Plan* masterPlan;
		Logger* log;
		RoleSet* roleSet;
		ISyncModul* syncModul;
		AuthorityManager* auth;
		IRoleAssignment* roleAssignment;
		ExpressionHandler* expressionHandler;
		list<IEngineModule*> mods;
		IPlanSelector* planSelector;
		IAlicaCommunication* communicator;
		IPlanner* planner;
		IAlicaClock* alicaClock;
		PartialAssignmentPool* pap;
		PlanBase* planBase;bool stepCalled;
		map<int, IConstraintSolver*> solver;
		IVariableSyncModule* variableSyncModule;

	private:

		/**
		 * Set to have the engine's main loop wait on a signal via MayStep
		 */
		bool stepEngine;
		/**
		 * Indicates whether the engine is shutting down.
		 */
		bool terminating;
		void setStepEngine(bool stepEngine);

		PlanRepository* planRepository;
		IPlanParser* planParser;
		IBehaviourPool* behaviourPool;
		ITeamObserver* teamObserver;

	};

	template<typename T>
	void AlicaEngine::abort(string msg, const T tail)
	{
		this->maySendMessages = false;
		stringstream ss;
		ss << msg << tail;
		AlicaEngine::abort(ss.str());
	}

} /* namespace Alica */

#endif /* ALICAENGINE_H_ */
