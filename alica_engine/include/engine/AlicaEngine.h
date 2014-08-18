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


	class AlicaEngine
	{
	public:
		static AlicaEngine* getInstance();
		bool init(IBehaviourCreator* bc, string roleSetName, string masterPlanName, string roleSetDir, bool stepEngine);
		bool shutdown();
		void start();
		bool getStepEngine();
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
		IPlanParser* getPlanParser();
		bool isTerminating() const;
		void setTerminating(bool terminating);
		void setStepCalled(bool stepCalled);
		bool getStepCalled() const;
		RoleSet* getRoleSet();
		IAlicaCommunication* getCommunicator();
		void setCommunicator(IAlicaCommunication * communicator);
		IPlanSelector* getPlanSelector();
		IPlanner* getPlanner();
		IAlicaClock* getIAlicaClock();
		void setIAlicaClock(IAlicaClock* clock);
		void doStep();
		void iterationComplete();

	protected:
		supplementary::SystemConfig* sc;
		Plan* masterPlan;
		Logger* log;
		RoleSet* roleSet;
		ISyncModul* syncModul;
		AuthorityManager* auth;
		IRoleAssignment* roleAssignment;
		list<IEngineModule*> mods;
		IPlanSelector* planSelector;
		IAlicaCommunication* communicator;
		IPlanner* planner;
		IAlicaClock* alicaClock;
		PlanBase* planBase;
		bool stepCalled;


	private:
		// private constructur/ destructor because of singleton
		AlicaEngine();
		~AlicaEngine();

		bool stepEngine;
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
		stringstream ss;
		ss << msg << tail;
		AlicaEngine::abort(ss.str());
	}

} /* namespace Alica */

#endif /* ALICAENGINE_H_ */
