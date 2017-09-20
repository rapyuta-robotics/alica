/*
 * TeamObserver.h
 *
 *  Created on: Jun 13, 2014
 *      Author: Stefan Jakob
 */

#ifndef TEAMOBSERVER_H_
#define TEAMOBSERVER_H_

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
		//event OnTeamChange OnTeamChangeEvent;
		void messageRecievedFrom(alica::IRobotID rid);
		virtual alica::IRobotID getOwnId();
		RobotEngineData* getRobotById(alica::IRobotID id);
		unique_ptr<list<RobotEngineData*> > getAvailableRobots();
		unique_ptr<list<shared_ptr<RobotProperties>> > getAvailableRobotProperties();
		unique_ptr<list<alica::IRobotID> > getAvailableRobotIds();
		shared_ptr<RobotProperties> getOwnRobotProperties();
		RobotEngineData* getOwnEngineData();
		int teamSize();
		unique_ptr<map<alica::IRobotID, shared_ptr<SimplePlanTree> > > getTeamPlanTrees();
		void init();
		void tick(shared_ptr<RunningPlan> root);
		void doBroadCast(list<long>& msg);
		int successesInPlan(Plan* plan);
		shared_ptr<SuccessCollection> getSuccessCollection(Plan* plan);
		void updateSuccessCollection(Plan* p, shared_ptr<SuccessCollection> sc);
		void ignoreRobot(alica::IRobotID rid);
		void unIgnoreRobot(alica::IRobotID rid);
		bool isRobotIgnored(alica::IRobotID rid);
		void notifyRobotLeftPlan(AbstractPlan* plan);
		virtual void handlePlanTreeInfo(shared_ptr<PlanTreeInfo> incoming);
		void close();

	private:
		EntryPoint* entryPointOfState(State* state);

	protected:
		mutex simplePlanTreeMutex;
		mutex successMark;
		list<RobotEngineData*> allOtherRobots;
		alica::IRobotID myId;
		RobotEngineData* me;
		shared_ptr<map<alica::IRobotID, shared_ptr<SimplePlanTree> > > simplePlanTrees;
		AlicaTime teamTimeOut;
		Logger* log;
		unordered_set<alica::IRobotID> ignoredRobots;
		AlicaEngine* ae;
		void cleanOwnSuccessMarks(shared_ptr<RunningPlan> root);
		shared_ptr<SimplePlanTree> sptFromMessage(alica::IRobotID robotId, list<long> ids);

	};

} /* namespace alica */

#endif /* TEAMOBSERVER_H_ */
