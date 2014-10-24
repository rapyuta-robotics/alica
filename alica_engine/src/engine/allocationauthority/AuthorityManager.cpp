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
		this->authorityPub = nullptr;
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
		//TODO::
//		this.rosNode = new Node("AlicaEngine");
//		this.rosNode.Subscribe("AllocationAuthorityInfo", HandleIncomingAuthorityMessage, 10);
//		authorityPub = new Publisher(this.rosNode, "AllocationAuthorityInfo", AllocationAuthorityInfo.TypeId, 2);
	}

	/**
	 * Closes this engine module
	 */
	void AuthorityManager::close()
	{
		//TODO::
//		if(this.rosNode!= null) this.rosNode.Close();
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
		for (int i = 0; i < this->queue.size(); i++)
		{
			if (authorityMatchesPlan(this->queue[i], p))
			{
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
		//Console.WriteLine("Sending Assignment: {0}",ass);
		auto eps = ass->getEntryPoints();
		for (int i = 0; i < eps->size(); i++)
		{
			it = EntryPointRobots();
			it.entrypoint = eps->at(i)->getId();
			auto robots = ass->getRobotsWorking(eps->at(i));
			for(int j = 0; j < robots->size(); j++)
			{
				it.robots.push_back(robots->at(j));
			}
			aai.entryPointRobots.push_back(it);
		}

		auto temp = p->getParent().lock()->getActiveState();
		aai.parentState = (p->getParent().expired() || temp == nullptr ? -1 : temp->getId());
		aai.planId = p->getPlan()->getId();
		aai.authority = this->ownID;
		aai.senderID = this->ownID;
		aai.planType = (p->getPlanType() == nullptr ? -1 : p->getPlanType()->getId());

		this->authorityPub->sendAllocationAuthority(aai);
	}

	bool AuthorityManager::authorityMatchesPlan(shared_ptr<AllocationAuthorityInfo> aai, shared_ptr<RunningPlan> p)
	{
		auto temp = p->getParent().lock()->getActiveState();
		if ((p->getParent().expired() && aai->parentState == -1)
				|| (!p->getParent().expired() && temp != nullptr
						&& aai->parentState == temp->getId()))
		{
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
