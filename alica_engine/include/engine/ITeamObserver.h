/*
 * ITeamObserver.h
 *
 *  Created on: Jun 13, 2014
 *      Author: Stefan Jakob
 */

#ifndef ITEAMOBSERVER_H_
#define ITEAMOBSERVER_H_

using namespace std;

#include <list>
#include <map>
#include <memory>

namespace alica
{
	class RunningPlan;
	class RobotEngineData;
	class RobotProperties;
	class SimplePlanTree;
	class SuccessCollection;
	class Plan;
	class AbstractPlan;
	struct PlanTreeInfo;

	class ITeamObserver
	{
	public:
		virtual ~ITeamObserver() {}
		virtual void init() = 0;
		virtual void close() = 0;
		virtual void tick(shared_ptr<RunningPlan> root) = 0;
		virtual unique_ptr<list<RobotEngineData*> > getAvailableRobots() = 0;
		virtual unique_ptr<list<RobotProperties*> > getAvailableRobotProperties() = 0;
		virtual unique_ptr<list<int> > getAvailableRobotIds() = 0;
		virtual int getOwnId() = 0;
		virtual int teamSize() = 0;
		virtual RobotEngineData* getOwnEngineData() = 0;
		virtual RobotEngineData* getRobotById(int id) = 0;
		virtual RobotProperties* getOwnRobotProperties() = 0;
		virtual unique_ptr<map<int, shared_ptr<SimplePlanTree> > > getTeamPlanTrees() = 0;
		virtual void handlePlanTreeInfo(shared_ptr<PlanTreeInfo> incoming) = 0;

		virtual int successesInPlan(Plan* p) = 0;
		virtual SuccessCollection* getSuccessCollection(Plan* p) = 0;
		virtual void updateSuccessCollection(Plan* p, SuccessCollection* sc) = 0;
		virtual void doBroadCast(list<long>& planmsg) = 0;

		virtual void ignoreRobot(int rid) = 0;
		virtual void unIgnoreRobot(int rid) = 0;
		virtual bool isRobotIgnored(int rid) = 0;

		//virtual void notifyTeamLeftPlan(AbstractPlan* p) = 0;
		//virtual void notifyILeftPlan(AbstractPlan* p) = 0;
		//both methods replaced by:
		virtual void notifyRobotLeftPlan(AbstractPlan* p) = 0;
		virtual void messageRecievedFrom(int rid) = 0;

		//event OnTeamChange OnTeamChangeEvent;

	};
}




#endif /* ITEAMOBSERVER_H_ */
