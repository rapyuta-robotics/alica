/*
 * ITeamObserver.h
 *
 *  Created on: Jun 13, 2014
 *      Author: Stefan Jakob
 */

#ifndef ITEAMOBSERVER_H_
#define ITEAMOBSERVER_H_

using namespace std;

#include "engine/IRobotID.h"

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
		virtual unique_ptr<list<shared_ptr<RobotProperties>> > getAvailableRobotProperties() = 0;
		virtual unique_ptr<list<alica::IRobotID> > getAvailableRobotIds() = 0;
		virtual alica::IRobotID getOwnId() = 0;
		virtual int teamSize() = 0;
		virtual RobotEngineData* getOwnEngineData() = 0;
		virtual RobotEngineData* getRobotById(alica::IRobotID id) = 0;
		virtual shared_ptr<RobotProperties> getOwnRobotProperties() = 0;
		virtual unique_ptr<map<alica::IRobotID, shared_ptr<SimplePlanTree> > > getTeamPlanTrees() = 0;
		virtual void handlePlanTreeInfo(shared_ptr<PlanTreeInfo> incoming) = 0;

		virtual int successesInPlan(Plan* p) = 0;
		virtual shared_ptr<SuccessCollection> getSuccessCollection(Plan* p) = 0;
		virtual void updateSuccessCollection(Plan* p, shared_ptr<SuccessCollection> sc) = 0;
		virtual void doBroadCast(list<long>& planmsg) = 0;

		virtual void ignoreRobot(alica::IRobotID rid) = 0;
		virtual void unIgnoreRobot(alica::IRobotID rid) = 0;
		virtual bool isRobotIgnored(alica::IRobotID rid) = 0;

		//virtual void notifyTeamLeftPlan(AbstractPlan* p) = 0;
		//virtual void notifyILeftPlan(AbstractPlan* p) = 0;
		//both methods replaced by:
		virtual void notifyRobotLeftPlan(AbstractPlan* p) = 0;
		virtual void messageRecievedFrom(alica::IRobotID rid) = 0;

		//event OnTeamChange OnTeamChangeEvent;

	};
}




#endif /* ITEAMOBSERVER_H_ */
