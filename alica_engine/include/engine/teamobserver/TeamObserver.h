/*
 * TeamObserver.h
 *
 *  Created on: Jun 13, 2014
 *      Author: Stefan Jakob
 */

#ifndef TEAMOBSERVER_H_
#define TEAMOBSERVER_H_

using namespace std;

#include <unordered_set>
#include <string>
#include <iostream>
#include <sstream>
#include <mutex>
#include <thread>
#include <memory>
#include <ctime>
#include <map>

#include "engine/ITeamObserver.h"

namespace alica
{

	class Logger;
	class AlicaEngine;
	class SystemConfig;
	class EntryPoint;
	class State;
	class SuccessCollection;

	class TeamObserver : public virtual ITeamObserver
	{
	public:
		TeamObserver();
		//TODO ICommunication interface destructor ==> unique_ptr ?
		virtual ~TeamObserver();
		//event OnTeamChange OnTeamChangeEvent;
		void messageRecievedFrom(int rid);
		virtual int getOwnId();
		RobotEngineData* getRobotById(int id);
		unique_ptr<list<RobotEngineData*> > getAvailableRobots();
		unique_ptr<list<RobotProperties*> > getAvailableRobotProperties();
		unique_ptr<list<int> > getAvailableRobotIds();
		RobotProperties* getOwnRobotProperties();
		RobotEngineData* getOwnEngineData();
		int teamSize();
		unique_ptr<map<int, shared_ptr<SimplePlanTree> > > getTeamPlanTrees();
		void init();
		void tick(shared_ptr<RunningPlan> root);
		void doBroadCast(list<long> msg);
		int successesInPlan(Plan* plan);
		SuccessCollection* getSuccessCollection(Plan* plan);
		void updateSuccessCollection(Plan* p, SuccessCollection* sc);
		void ignoreRobot(int rid);
		void unIgnoreRobot(int rid);
		bool isRobotIgnored(int rid);
		void notifyRobotLeftPlan(AbstractPlan* plan);
		virtual void handlePlanTreeInfo(shared_ptr<PlanTreeInfo> incoming);
		void close();

	private:
		EntryPoint* entryPointOfState(State* state);

	protected:
		static mutex simplePlanTreeMutex;
		list<RobotEngineData*> allOtherRobots;
		int myId;
		RobotEngineData* me;
		//TODO ICommunication interface
		//C#
//		protected IntPtr planTreePublisher;
//		protected Node rosNode;
		shared_ptr<map<int, shared_ptr<SimplePlanTree> > > simplePlanTrees;
		unsigned long teamTimeOut;
		Logger* log;
		unordered_set<int> ignoredRobots;
		AlicaEngine* ae;
		void cleanOwnSuccessMarks(shared_ptr<RunningPlan> root);
		shared_ptr<SimplePlanTree> sptFromMessage(int robotId, list<long> ids);

	};

} /* namespace alica */

#endif /* TEAMOBSERVER_H_ */
