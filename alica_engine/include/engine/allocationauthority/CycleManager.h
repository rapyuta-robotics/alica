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
#include "engine/IAlicaClock.h"

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
	class AlicaEngine;

	/**
	 * Responsibile for detecting cycles in assignment updates and reactions to these
	 */
	class CycleManager
	{
	public:
		CycleManager(AlicaEngine* ae, RunningPlan* p);
		virtual ~CycleManager();
		void update();
		bool isOverridden();
		bool setAssignment(shared_ptr<RunningPlan> r);
		bool mayDoUtilityCheck();
		void setNewAllocDiff(RunningPlan* curP, AllocationDifference* aldif);
		void setNewAllocDiff(RunningPlan* curP, Assignment* oldAss, Assignment* newAss, AllocationDifference::Reason reas);
		void handleAuthorityInfo(shared_ptr<AllocationAuthorityInfo> aai);
		bool needsSending();
		void sent();
		bool haveAuthority();


	protected:
		AlicaEngine* ae;
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
		alicaTime overrideTimestamp;
		double intervalIncFactor;
		double intervalDecFactor;
		static alicaTime minimalOverrideTimeInterval;
		static alicaTime maximalOverrideTimeInterval;
		static alicaTime overrideShoutInterval;
		static alicaTime overrideWaitInterval;
		alicaTime overrideShoutTime;
		static int historySize;
		CycleState state;
		RunningPlan* rp;
		shared_ptr<AllocationAuthorityInfo> fixedAllocation;
		bool detectAllocationCycle();
	};

} /* namespace supplementary */

#endif /* CYCLEMANAGER_H_ */
