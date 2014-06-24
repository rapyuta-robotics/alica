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
		const string& getRobotName() const;
		void setRobotName(const string& robotName);
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

	protected:
		supplementary::SystemConfig* sc;
		Plan* masterPlan;
		Logger* log;
		string robotName;
		RoleSet* roleSet;
		ISyncModul* syncModul;
		AuthorityManager* auth;
		IRoleAssignment* roleAssignment;

	private:
		// private constructur/ destructor because of singleton
		AlicaEngine();
		~AlicaEngine();

		bool stepEngine;
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
