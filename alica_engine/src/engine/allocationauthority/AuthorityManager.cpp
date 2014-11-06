/*
 * AuthorityManager.cpp
 *
 *  Created on: Jun 17, 2014
 *      Author: Paul Panin
 */

#include "engine/allocationauthority/AuthorityManager.h"
#include "engine/allocationauthority/CycleManager.h"
#include "engine/model/State.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/PlanType.h"
#include "engine/Assignment.h"
#include "engine/collections/AssignmentCollection.h"
#include "engine/model/EntryPoint.h"

namespace alica
{
	/**
	 * Constructor
	 */
	AuthorityManager::AuthorityManager(AlicaEngine* ae)
	{
		this->ae = ae;
		this->ownID = 0;
	}

	AuthorityManager::~AuthorityManager()
	{
	}

	/**
	 * Initialises this engine module
	 */
	void AuthorityManager::init()
	{
		this->ownID = ae->getTeamObserver()->getOwnId();
	}

	/**
	 * Closes this engine module
	 */
	void AuthorityManager::close()
	{

	}

	/**
	 * Message Handler
	 * param name = aai A AllocationAthorityInfo
	 */
	void AuthorityManager::handleIncomingAuthorityMessage(shared_ptr<AllocationAuthorityInfo> aai)
	{
		if (ae->getTeamObserver()->isRobotIgnored(aai->senderID))
		{
			return;
		}
		if (aai->senderID != this->ownID)
		{
			ae->getTeamObserver()->messageRecievedFrom(aai->senderID);
			if (aai->senderID > this->ownID)
			{
				for (EntryPointRobots epr : aai->entryPointRobots)
				{
					for (int rid : epr.robots)
					{
						if (rid != this->ownID)
						{
							ae->getTeamObserver()->messageRecievedFrom(rid);
						}
					}
				}
			}
		}
		{
			lock_guard<mutex> lock(mu);
			this->queue.push_back(aai);
		}
	}
	/**
	 * Cyclic tick function, called by the plan base every iteration
	 */
	void AuthorityManager::tick(shared_ptr<RunningPlan> p)
	{
		lock_guard<mutex> lock(mu);
		processPlan(p);
		this->queue.clear();
	}

	void AuthorityManager::processPlan(shared_ptr<RunningPlan> p)
	{
		if (p == nullptr || p->isBehaviour())
		{
			return;
		}
		if (p->isBehaviour())
		{
			return;
		}
		if (p->getCycleManagement()->needsSending())
		{
			sendAllocation(p);
			p->getCycleManagement()->sent();
		}
#ifdef AM_DEBUG
		cout << "AM: outside for " << endl;
#endif
		for (int i = 0; i < this->queue.size(); i++)
		{
#ifdef AM_DEBUG
			cout << "AM: inside for " << endl;
#endif
			if (authorityMatchesPlan(this->queue[i], p))
			{
#ifdef AM_DEBUG
				cout << "AM: p->getCycleManagement()->handleAuthorityInfo(this->queue[i]);" << endl;
#endif
				p->getCycleManagement()->handleAuthorityInfo(this->queue[i]);
				this->queue.erase(this->queue.begin() + i);
				i--;
			}
		}
		for (shared_ptr<RunningPlan> c : *p->getChildren())
		{
			processPlan(c);
		}

	}
	/**
	 * Sends an AllocationAuthorityInfo message containing the assignment of p
	 */
	void AuthorityManager::sendAllocation(shared_ptr<RunningPlan> p)
	{
		if (!this->ae->isMaySendMessages())
		{
			return;
		}
		AllocationAuthorityInfo aai = AllocationAuthorityInfo();

		EntryPointRobots it;
		shared_ptr<Assignment> ass = p->getAssignment();
		for (int i = 0; i < ass->getEntryPointCount(); i++)
		{
			it = EntryPointRobots();
			it.entrypoint = ass->getEpRobotsMapping()->getEp(i)->getId();
			auto robots = ass->getRobotsWorking(ass->getEpRobotsMapping()->getEp(i));
			for (int j = 0; j < robots->size(); j++)
			{
				it.robots.push_back(robots->at(j));
			}
			aai.entryPointRobots.push_back(it);
		}

		auto shared = p->getParent().lock();
		//TODO id is always 1359155945 which is not in etc/plans
		aai.parentState = (
				(p->getParent().expired() || shared->getActiveState() == nullptr) ? -1 :
						shared->getActiveState()->getId());
		cout << "AM: aai.parentState " << aai.parentState << endl;
		aai.planId = p->getPlan()->getId();
		aai.authority = this->ownID;
		aai.senderID = this->ownID;
		aai.planType = (p->getPlanType() == nullptr ? -1 : p->getPlanType()->getId());

		this->ae->getCommunicator()->sendAllocationAuthority(aai);
	}

	bool AuthorityManager::authorityMatchesPlan(shared_ptr<AllocationAuthorityInfo> aai, shared_ptr<RunningPlan> p)
	{
		auto shared = p->getParent().lock();

		if (!p->getParent().expired())
		{
			cout << "AM: expired " << !p->getParent().expired() << " aai->parentState == -1 "
					<< (aai->parentState == -1) << " shared->getActiveState() != nullptr "
					<< (shared->getActiveState() != nullptr)
					<< " aai->parentState == shared->getActiveState()->getId() "
					<< (aai->parentState == shared->getActiveState()->getId()) << endl;
		}
		else
		{
			cout << "AM: expired " << aai->parentState << endl;
		}
		if ((p->getParent().expired() && aai->parentState == -1)
				|| (!p->getParent().expired() && shared->getActiveState() != nullptr
						&& aai->parentState == shared->getActiveState()->getId()))
		{
			cout << "AM: matches" << endl;
			if (p->getPlan()->getId() == aai->planId)
			{
				return true;
			}
			else if (aai->planType != -1 && p->getPlanType() != nullptr && p->getPlanType()->getId() == aai->planType)
			{
				return true;
			}
		}
		return false;
	}

}
