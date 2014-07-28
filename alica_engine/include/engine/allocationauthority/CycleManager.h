/*
 * CycleManager.h
 *
 *  Created on: Jul 10, 2014
 *      Author: Paul Panin
 */

#ifndef CYCLEMANAGER_H_
#define CYCLEMANAGER_H_
#define CM_DEBUG

using namespace std;

#include <vector>
#include <thread>
#include <mutex>


#include "engine/allocationauthority/AllocationDifference.h"
namespace supplementary
{
	class SystemConfig;
}
namespace alica
{
	class RunningPlan;
	class PlanRepository;
	struct AllocationAuthorityInfo;
	class Assignment;

	class CycleManager
	{
	public:
		CycleManager(RunningPlan* p);
		virtual ~CycleManager();
		void update();
		bool isOverridden();
		bool setAssignment(RunningPlan* r);
		bool mayDoUtilityCheck();
		void setNewAllocDiff(RunningPlan* curP, AllocationDifference* aldif);
		void setNewAllocDiff(RunningPlan* curP, Assignment* oldAss, Assignment* newAss, AllocationDifference::Reason reas);
		void handleAuthorityInfo(AllocationAuthorityInfo* aai);
		bool needsSending();
		void sent();
		bool haveAuthority();


	protected:
		mutex allocationHistoryMutex;
		static supplementary::SystemConfig* sc;
		static int maxAllocationCycles;
		static bool enabled;
		vector<AllocationDifference*> allocationHistory;
		PlanRepository* pr;
		int newestAllocationDifference;
		int myID;
		enum CycleState
		{
			observing, overridden, overriding
		};
		unsigned long overrideTimestamp;
		double intervalIncFactor;
		double intervalDecFactor;
		static unsigned long minimalOverrideTimeInterval;
		static unsigned long maximalOverrideTimeInterval;
		static unsigned long overrideShoutInterval;
		static unsigned long overrideWaitInterval;
		unsigned long overrideShoutTime;
		static int historySize;
		CycleState state;
		RunningPlan* rp;
		AllocationAuthorityInfo* fixedAllocation;
		bool detectAllocationCycle();
	};

} /* namespace supplementary */

#endif /* CYCLEMANAGER_H_ */
