/*
 * CycleManager.h
 *
 *  Created on: Jul 10, 2014
 *      Author: Paul Panin
 */

#ifndef CYCLEMANAGER_H_
#define CYCLEMANAGER_H_

//#define CM_DEBUG


#include <vector>
#include <thread>
#include <mutex>


#include "engine/allocationauthority/AllocationDifference.h"
#include "engine/IAlicaClock.h"

using namespace std;
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
		bool setAssignment();
		bool mayDoUtilityCheck();
		void setNewAllocDiff(AllocationDifference* aldif);
		void setNewAllocDiff(shared_ptr<Assignment> oldAss, shared_ptr<Assignment> newAss, AllocationDifference::Reason reas);
		void handleAuthorityInfo(shared_ptr<AllocationAuthorityInfo> aai);
		bool needsSending();
		void sent();
		bool haveAuthority();


	protected:
		AlicaEngine* ae;
		mutex allocationHistoryMutex;
		supplementary::SystemConfig* sc;
		int maxAllocationCycles;
		bool enabled;
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
		alicaTime minimalOverrideTimeInterval;
		alicaTime maximalOverrideTimeInterval;
		alicaTime overrideShoutInterval;
		alicaTime overrideWaitInterval;
		alicaTime overrideShoutTime;
		int historySize;
		CycleState state;
		RunningPlan* rp;
		shared_ptr<AllocationAuthorityInfo> fixedAllocation;
		bool detectAllocationCycle();
	};

} /* namespace supplementary */

#endif /* CYCLEMANAGER_H_ */
