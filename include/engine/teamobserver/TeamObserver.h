/*
 * TeamObserver.h
 *
 *  Created on: Jun 13, 2014
 *      Author: stefan
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

#include "engine/ITeamObserver.h"

namespace alica
{

	class Logger;
	class AlicaEngine;
	class SystemConfig;

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
		unique_ptr<list<RobotEngineData*> > GetAvailableRobots();
		unique_ptr<list<RobotProperties*> > GetAvailableRobotProperties();
		unique_ptr<list<int> > GetAvailableRobotIds();
		RobotProperties* GetOwnRobotProperties();
		RobotEngineData* GetOwnEngineData();
		int teamSize();
		unique_ptr<map<int, SimplePlanTree*> > GetTeamPlanTrees();
		void init();

	protected:
		static mutex simplePlanTreeMutex;
		list<RobotEngineData*> allOtherRobots;
		int myId;
		RobotEngineData* me;
		//TODO ICommunication interface
		//C#
//		protected IntPtr planTreePublisher;
//		protected Node rosNode;
		map<int, SimplePlanTree*> simplePlanTrees;
		unsigned long teamTimeOut;
		Logger* log;
		unordered_set<int> ignoredRobots;
		AlicaEngine* ae;
		void cleanOwnSuccessMarks(RunningPlan* root);

	};

} /* namespace alica */

#endif /* TEAMOBSERVER_H_ */
