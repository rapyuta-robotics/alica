/*
 * CycleManager.cpp
 *
 *  Created on: Jul 10, 2014
 *      Author:  Paul Panin
 */

#include "engine/allocationauthority/CycleManager.h"

#include <SystemConfig.h>
#include "engine/AlicaEngine.h"
#include "engine/allocationauthority/AllocationDifference.h"
#include "engine/ITeamObserver.h"
#include "engine/IAlicaClock.h"

namespace alica
{
	supplementary::SystemConfig* CycleManager::sc = supplementary::SystemConfig::getInstance();
	int CycleManager::maxAllocationCycles = (*sc)["alica"]->get<int>("Alica", "CycleDetection", "CycleCount");
	bool CycleManager::enabled = (*sc)["alica"]->get<bool>("Alica", "CycleDetection", "Enabled");
	unsigned long CycleManager::minimalOverrideTImeInterval = (*sc)["alica"]->get<unsigned long>(
			"Alica", "CycleDetection", "MinimalAuthorityTimeInterval") * 1000000;
	unsigned long CycleManager::maximalOverrideTimeInterval = (*sc)["alica"]->get<unsigned long>(
			"Alica", "CycleDetection", "MaximalAuthorityTimeInterval") * 1000000;
	unsigned long CycleManager::overrideShoutInterval = (*sc)["alica"]->get<unsigned long>("Alica", "CycleDetection",
																							"MessageTimeInterval")
			* 1000000;
	unsigned long CycleManager::overrideWaitInterval = (*sc)["alica"]->get<unsigned long>("Alica", "CycleDetection",
																							"MessageWaitTimeInterval")
			* 1000000;
	int CycleManager::historySize = (*sc)["alica"]->get<int>("Alica", "CycleDetection", "HistorySize");

	CycleManager::CycleManager(RunningPlan* p)
	{
		this->intervalIncFactor = (*sc)["alica"]->get<double>("Alica", "CycleDetection", "IntervalIncreaseFactor");
		this->intervalDecFactor = (*sc)["alica"]->get<double>("Alica", "CycleDetection", "IntervalDecreaseFactor");
		this->allocationHistory = vector<AllocationDifference*>(this->historySize);
		for (int i = 0; i < this->allocationHistory.size(); i++)
		{
			this->allocationHistory[i] = new AllocationDifference();
		}
		this->newestAllocationDifference = 0;
		this->state = CycleState::observing;
		this->rp = p;
		this->myID = AlicaEngine::getInstance()->getTeamObserver()->getOwnId();
		this->pr = AlicaEngine::getInstance()->getPlanRepository();
	}

	CycleManager::~CycleManager()
	{
	}

	void CycleManager::update()
	{

	}
	bool CycleManager::isOverridden()
	{
		return this->state==CycleState::overridden && this->fixedAllocation != nullptr;
	}

	void alica::CycleManager::setNewAllocDiff(RunningPlan* curP, AllocationDifference* aldif)
	{
	}

	void alica::CycleManager::setNewAllocDiff(RunningPlan* curP, Assignment* oldAss, Assignment* newAss,
												AllocationDifference::Reason reas)
	{
	}

	void alica::CycleManager::handleAuthorityInfo(AllocationAuthorityInfo* aai)
	{
	}

	bool alica::CycleManager::needsSending()
	{
		return this->state==CycleState::overriding && (this->overrideShoutTime + overrideShoutInterval < AlicaEngine::getInstance()->getIAlicaClock()->now());
	}

	void alica::CycleManager::sent()
	{
		this->overrideShoutTime = AlicaEngine::getInstance()->getIAlicaClock()->now();
	}

	bool alica::CycleManager::haveAuthority()
	{
		return this->state == CycleState::overriding;
	}

	bool CycleManager::setAssignment(RunningPlan* r)
	{
		//TODO:
	}
	bool CycleManager::mayDoUtilityCheck()
	{
		return this->state != CycleState::overridden;
	}

	bool CycleManager::detectAllocationCycle()
	{
	}

} /* namespace supplementary */
