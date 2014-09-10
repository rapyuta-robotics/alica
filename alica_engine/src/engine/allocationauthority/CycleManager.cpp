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
#include "engine/model/EntryPoint.h"
#include "engine/Assignment.h"
#include "engine/allocationauthority/EntryPointRobotPair.h"
#include "engine/RunningPlan.h"
#include "engine/model/Plan.h"
#include "engine/model/AbstractPlan.h"
#include "engine/containers/AllocationAuthorityInfo.h"
#include "engine/model/PlanType.h"
#include "engine/PlanRepository.h"
#include "engine/containers/EntryPointRobots.h"
#include "engine/collections/StateCollection.h"

namespace alica
{
	supplementary::SystemConfig* CycleManager::sc = supplementary::SystemConfig::getInstance();
	int CycleManager::maxAllocationCycles = (*sc)["Alica"]->get<int>("Alica", "CycleDetection", "CycleCount");
	bool CycleManager::enabled = (*sc)["Alica"]->get<bool>("Alica", "CycleDetection", "Enabled");
	unsigned long CycleManager::minimalOverrideTimeInterval = (*sc)["Alica"]->get<unsigned long>(
			"Alica", "CycleDetection", "MinimalAuthorityTimeInterval") * 1000000;
	unsigned long CycleManager::maximalOverrideTimeInterval = (*sc)["Alica"]->get<unsigned long>(
			"Alica", "CycleDetection", "MaximalAuthorityTimeInterval") * 1000000;
	unsigned long CycleManager::overrideShoutInterval = (*sc)["Alica"]->get<unsigned long>("Alica", "CycleDetection",
																							"MessageTimeInterval")
			* 1000000;
	unsigned long CycleManager::overrideWaitInterval = (*sc)["Alica"]->get<unsigned long>("Alica", "CycleDetection",
																							"MessageWaitTimeInterval")
			* 1000000;
	int CycleManager::historySize = (*sc)["Alica"]->get<int>("Alica", "CycleDetection", "HistorySize");

	CycleManager::CycleManager(RunningPlan* p)
	{
		this->intervalIncFactor = (*sc)["Alica"]->get<double>("Alica", "CycleDetection", "IntervalIncreaseFactor");
		this->intervalDecFactor = (*sc)["Alica"]->get<double>("Alica", "CycleDetection", "IntervalDecreaseFactor");
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
		this->overrideTimestamp = 0;
		this->overrideShoutTime = 0;
		this->fixedAllocation = nullptr;
	}

	CycleManager::~CycleManager()
	{
	}

	void CycleManager::update()
	{
		if (!enabled)
		{
			return;
		}
		if (this->rp->isBehaviour())
		{
			return;
		}
		if (this->state == CycleState::observing)
		{
			if (detectAllocationCycle())
			{
				cout << "Cycle Detected!" << endl;

				this->state = CycleState::overriding;
				this->rp->getPlan()->setAuthorityTimeInterval(
						min(maximalOverrideTimeInterval,
							(unsigned long)(this->rp->getPlan()->getAuthorityTimeInterval() * intervalIncFactor)));
				this->overrideShoutTime = 0;
				cout << "Assuming Authority for " << this->rp->getPlan()->getAuthorityTimeInterval() / 1000000000.0
						<< "sec!" << endl;
				this->overrideTimestamp = AlicaEngine::getInstance()->getIAlicaClock()->now();
			}
			else
			{
				this->rp->getPlan()->setAuthorityTimeInterval(
						max(minimalOverrideTimeInterval,
							(unsigned long)(this->rp->getPlan()->getAuthorityTimeInterval() * intervalDecFactor)));
			}
		}
		else
		{
			if (this->state == CycleState::overriding
					&& this->overrideTimestamp + this->rp->getPlan()->getAuthorityTimeInterval()
							< AlicaEngine::getInstance()->getIAlicaClock()->now())
			{
#ifdef CM_DEBUG
				cout << "Resume Observing!" << endl;
#endif
				this->state = CycleState::observing;
				this->fixedAllocation = nullptr;
			}
			else if (this->state == CycleState::overridden
					&& this->overrideShoutTime + this->rp->getPlan()->getAuthorityTimeInterval()
							< AlicaEngine::getInstance()->getIAlicaClock()->now())
			{
#ifdef CM_DEBUG
				cout << "Resume Observing!" << endl;
#endif
				this->state = CycleState::observing;
				this->fixedAllocation = nullptr;

			}
		}
	}
	bool CycleManager::isOverridden()
	{
		return this->state == CycleState::overridden && this->fixedAllocation != nullptr;
	}

	void alica::CycleManager::setNewAllocDiff(RunningPlan* curP, AllocationDifference* aldif)
	{
		if (!enabled)
		{
			return;
		}
		lock_guard<mutex> lock(this->allocationHistoryMutex);
		try
		{
			this->newestAllocationDifference = (this->newestAllocationDifference + 1) % this->allocationHistory.size();
			this->allocationHistory[this->newestAllocationDifference] = aldif;
		}
		catch (exception &e)
		{
			cerr << "Exception in Alloc Difference Calculation:" << endl;
			cerr << e.what();
		}
	}

	void alica::CycleManager::setNewAllocDiff(RunningPlan* curP, Assignment* oldAss, Assignment* newAss,
												AllocationDifference::Reason reas)
	{
		if (!enabled)
		{
			return;
		}
		this->rp = curP;
		if (oldAss == nullptr)
		{
			return;
		}
		lock_guard<mutex> lock(this->allocationHistoryMutex);
		try
		{
			this->newestAllocationDifference = (this->newestAllocationDifference + 1) % this->allocationHistory.size();
			this->allocationHistory[this->newestAllocationDifference]->reset();

			for (EntryPoint* ep : (*oldAss->getEntryPoints()))
			{

				auto newRobots = newAss->getRobotsWorking(ep);
				auto oldRobots = oldAss->getRobotsWorking(ep);
				for (int oldId : (*oldRobots))
				{
					// C# newRobots->size() == nullptr
					if (newRobots->size() == 0 || find(newRobots->begin(), newRobots->end(), oldId) == newRobots->end())
					{
						this->allocationHistory[this->newestAllocationDifference]->getSubtractions().push_back(
								new EntryPointRobotPair(ep, oldId));
					}
				}
				if (newRobots != nullptr)
				{
					for (int newId : (*newRobots))
					{
						if (find(oldRobots->begin(), oldRobots->end(), newId) == oldRobots->end())
						{
							this->allocationHistory[this->newestAllocationDifference]->getAdditions().push_back(
									new EntryPointRobotPair(ep, newId));
						}
					}
				}

			}
			this->allocationHistory[this->newestAllocationDifference]->setReason(reas);
		}
		catch (exception &e)
		{
			cerr << "Exception in Alloc Difference Calculation:" << endl;
			cerr << e.what();

		}
	}

	void alica::CycleManager::handleAuthorityInfo(shared_ptr<AllocationAuthorityInfo> aai)
	{
		if (!enabled)
		{
			return;
		}
		long rid = aai->authority;
		if (rid == myID)
		{
			return;
		}
		if (rid > myID)
		{
			this->state = CycleState::overridden;
			this->overrideShoutTime = AlicaEngine::getInstance()->getIAlicaClock()->now();
			this->fixedAllocation = aai;
		}
		else
		{
			cout << "Rcv: Rejecting Authority!" << endl;
			if (this->state != CycleState::overriding)
			{
				this->state = CycleState::overriding;
				this->rp->getPlan()->setAuthorityTimeInterval(
						min(maximalOverrideTimeInterval,
							(unsigned long)(this->rp->getPlan()->getAuthorityTimeInterval() * intervalIncFactor)));
				this->overrideTimestamp = AlicaEngine::getInstance()->getIAlicaClock()->now();
				this->overrideShoutTime = 0;
			}
		}
	}

	bool alica::CycleManager::needsSending()
	{
		return this->state == CycleState::overriding
				&& (this->overrideShoutTime + overrideShoutInterval
						< AlicaEngine::getInstance()->getIAlicaClock()->now());
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
#ifdef CM_DEBUG
		cout << "Setting new Assignment " << rp->getPlan()->getName() << "!" << endl;
#endif
		EntryPoint* myEntryPoint = nullptr;
		if (this->fixedAllocation == nullptr)
		{
			return false;
		}
		bool modifiedSelf = false;
		bool modified = false;
		if (this->fixedAllocation->planId != rp->getPlan()->getId())
		{
			if (rp->getPlanType()->getId() != this->fixedAllocation->planType)
			{
				return false;
			}
			Plan* newPlan = nullptr;
			for (Plan* p : rp->getPlanType()->getPlans())
			{
				if (p->getId() == this->fixedAllocation->planId)
				{
					newPlan = p;
					rp->setPlan(p);
					break;
				}
			}
			rp->setAssignment(new Assignment(newPlan, this->fixedAllocation));
			for (EntryPointRobots epr : this->fixedAllocation->entryPointRobots)
			{
				if (find(epr.robots.begin(), epr.robots.end(), myID) != epr.robots.end())
				{
					myEntryPoint = pr->getEntryPoints().at(epr.entrypoint);
				}
			}

			modifiedSelf = true;

		}
		else
		{
			for (EntryPointRobots epr : this->fixedAllocation->entryPointRobots)
			{
				for (int robot : epr.robots)
				{
					EntryPoint* e = pr->getEntryPoints().at(epr.entrypoint);
					bool changed = rp->getAssignment()->updateRobot(robot, e);
					if (changed)
					{
						if (robot == myID)
						{
							modifiedSelf = true;
							myEntryPoint = e;
						}
						else
						{
							modified = true;
						}
					}
				}
			}
		}
		if (modifiedSelf)
		{
			rp->setOwnEntryPoint(myEntryPoint);
			rp->deactivateChildren();
			rp->clearChildren();
			rp->clearFailedChildren();
			rp->setAllocationNeeded(true);

		}
		else
		{
			if (rp->getActiveState() != nullptr)
			{
				auto robotsJoined = rp->getAssignment()->getRobotStateMapping()->getRobotsInState(rp->getActiveState());
				for (RunningPlan* c : rp->getChildren())
				{
					c->limitToRobots(robotsJoined);
				}
			}
		}
#ifdef CM_DEBUG
		if (!modifiedSelf)
		{
			cout << "But no change!" << endl;
		}
#endif
		return modifiedSelf || modified;
	}

	bool CycleManager::mayDoUtilityCheck()
	{
		return this->state != CycleState::overridden;
	}

	bool CycleManager::detectAllocationCycle()
	{
		int cyclesFound = 0;
		int count = 0;
		AllocationDifference* utChange = nullptr;
		AllocationDifference* temp = new AllocationDifference();
		lock_guard<mutex> lock(this->allocationHistoryMutex);
		{
			for (int i = this->newestAllocationDifference; count < this->allocationHistory.size(); i--)
			{
				count++;
				if (i < 0)
				{
					i = this->allocationHistory.size() - 1;
				}

				if (this->allocationHistory[i]->getReason() == AllocationDifference::Reason::utility)
				{
					if (utChange != nullptr)
					{
						return false;
					}
					utChange = this->allocationHistory[i];
					temp->reset();
					temp->applyDifference(utChange);
				}
				else
				{
					if (this->allocationHistory[i]->getReason() == AllocationDifference::Reason::empty)
					{
						return false;
					}
					if (utChange == nullptr)
					{
						continue;
					}
					temp->applyDifference(this->allocationHistory[i]);
					if (temp->isEmpty())
					{
						cyclesFound++;
						if (cyclesFound > maxAllocationCycles)
						{
							for (int k = 0; k < this->allocationHistory.size(); k++)
							{
								this->allocationHistory[k]->reset();
							}
							return true;
						}
						utChange = nullptr;
					}

				}
			}

		}
		return false;
	}

} /* namespace supplementary */
