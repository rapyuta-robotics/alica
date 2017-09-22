#pragma once
//#define TO_DEBUG

#include "engine/IRobotID.h"
#include "engine/ITeamObserver.h"
#include "engine/IAlicaClock.h"
#include "engine/IRoleAssignment.h"

#include <unordered_set>
#include <string>
#include <iostream>
#include <sstream>
#include <mutex>
#include <thread>
#include <memory>
#include <ctime>
#include <map>

using namespace std;
namespace alica
{

	class Logger;
	class AlicaEngine;
	class EntryPoint;
	class State;
	class SuccessCollection;
	class AlicaEngine;

	/**
	 * The TeamObserver manages communication with the team. Thus it sends and receives PlanTreeInfo messages.
	 * Specialized Modules may communicate through other means.
	 */
	class TeamObserver : public virtual ITeamObserver
	{
	public:
		TeamObserver(AlicaEngine* ae);
		virtual ~TeamObserver();

		void tick(shared_ptr<RunningPlan> root);
		void doBroadCast(list<long>& msg);

		unique_ptr<map<const alica::IRobotID*, shared_ptr<SimplePlanTree> > > getTeamPlanTrees();

		int successesInPlan(Plan* plan);
		shared_ptr<SuccessCollection> getSuccessCollection(Plan* plan);
		void updateSuccessCollection(Plan* p, shared_ptr<SuccessCollection> sc);

		void notifyRobotLeftPlan(AbstractPlan* plan);
		virtual void handlePlanTreeInfo(shared_ptr<PlanTreeInfo> incoming);
		void close();

	private:
		EntryPoint* entryPointOfState(State* state);

	protected:
		AlicaEngine* ae;
		Logger* log;
		const alica::IRobotID* myId;
		RobotEngineData* me;

		mutex simplePlanTreeMutex;
		mutex successMark;

		shared_ptr<map<const alica::IRobotID*, shared_ptr<SimplePlanTree> > > simplePlanTrees;

		void cleanOwnSuccessMarks(shared_ptr<RunningPlan> root);
		shared_ptr<SimplePlanTree> sptFromMessage(alica::IRobotID robotId, list<long> ids);

	};

} /* namespace alica */
