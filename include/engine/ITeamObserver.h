/*
 * ITeamObserver.h
 *
 *  Created on: Jun 13, 2014
 *      Author: stefan
 */

#ifndef ITEAMOBSERVER_H_
#define ITEAMOBSERVER_H_

using namespace std;

#include <list>
#include <map>

namespace alica
{
	class RunningPlan;
	class RobotEngineData;
	class RobotProperties;
	class SimplePlanTree;
	class SuccesCollection;
	class Plan;
	class AbstractPlan;

	class ITeamObserver
	{
	public:
		virtual ~ITeamObserver() {}
		virtual void init() = 0;
		virtual void close() = 0;
		virtual void tick(RunningPlan* root) = 0;
		virtual list<RobotEngineData*> getAvailableRobots() = 0;
		virtual list<RobotProperties*> getAvailableRobotProperties() = 0;
		virtual list<int> getAvailableRobotIds() = 0;
		virtual int getOwnId() = 0;
		virtual int teamSize() = 0;
		virtual RobotEngineData* getOwnEngineData() = 0;
		virtual RobotEngineData* getRobotById(int id) = 0;
		virtual RobotProperties* getOwnRobotProperties() = 0;
		virtual map<int, SimplePlanTree*> getTeamPlanTrees() = 0;

		virtual int successesInPlan(Plan* p) = 0;
		virtual SuccesCollection getSuccessCollection(Plan* p) = 0;
		virtual void updateSuccessCollection(Plan* p, SuccesCollection* sc) = 0;

		virtual void doBroadCast(list<long> planmsg) = 0;

		virtual void ignoreRobot(int rid) = 0;
		virtual void unIgnoreRobot(int rid) = 0;
		virtual bool isRobotIgnored(int rid) = 0;

		virtual void notifyTeamLeftPlan(AbstractPlan* p) = 0;
		virtual void notifyILeftPlan(AbstractPlan* p) = 0;
		virtual void messageRecievedFrom(int rid) = 0;

		//event OnTeamChange OnTeamChangeEvent;

	};
}




#endif /* ITEAMOBSERVER_H_ */
