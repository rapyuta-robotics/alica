#pragma once
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
		virtual void close() = 0;
		virtual void tick(shared_ptr<RunningPlan> root) = 0;

		virtual unique_ptr<map<const alica::IRobotID*, shared_ptr<SimplePlanTree> > > getTeamPlanTrees() = 0;
		virtual void handlePlanTreeInfo(shared_ptr<PlanTreeInfo> incoming) = 0;

		virtual int successesInPlan(Plan* p) = 0;
		virtual shared_ptr<SuccessCollection> getSuccessCollection(Plan* p) = 0;
		virtual void updateSuccessCollection(Plan* p, shared_ptr<SuccessCollection> sc) = 0;
		virtual void doBroadCast(list<long>& planmsg) = 0;

		virtual void notifyRobotLeftPlan(AbstractPlan* p) = 0;
	};
}
