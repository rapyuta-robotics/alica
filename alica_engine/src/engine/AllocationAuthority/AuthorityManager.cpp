/*
 * AuthorityManager.cpp
 *
 *  Created on: Jun 17, 2014
 *      Author: Paul Panin
 */

#include "engine/allocationauthority/AuthorityManager.h"

namespace alica
{
	AuthorityManager::AuthorityManager()
	{
		this->ae = AlicaEngine::getInstance();
	}

	AuthorityManager::~AuthorityManager()
	{
	}

	void AuthorityManager::init()
	{
		this->ownID = AlicaEngine::getInstance()->getTeamObserver()->getOwnId();
		//TODO::
//		this.rosNode = new Node("AlicaEngine");
//		this.rosNode.Subscribe("AllocationAuthorityInfo", HandleIncomingAuthorityMessage, 10);
//		authorityPub = new Publisher(this.rosNode, "AllocationAuthorityInfo", AllocationAuthorityInfo.TypeId, 2);
	}
	void AuthorityManager::close()
	{
		//TODO::
//		if(this.rosNode!= null) this.rosNode.Close();
	}

	/**
	 * Message Handler
	 * param name = aai
	 */
	void AuthorityManager::handleIncomingAuthorityMessage(AllocationAuthorityInfo aai)
	{
		if (ae->getTeamObserver()->isRobotIgnored(aai.senderID))
		{
			return;
		}
		if (aai.senderID != this->ownID)
		{
			ae->getTeamObserver()->messageRecievedFrom(aai.senderID);
			for (EntryPointRobots epr : aai.entryPointRobots)
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
	void AuthorityManager::tick(RunningPlan* p)
	{
		lock_guard<mutex> lock(mu);
		processPlan(p);
		this->queue.clear();
	}

	void AuthorityManager::processPlan(RunningPlan* p)
	{
		if (p == nullptr || p->isBehaviour())
		{
			return;
		}
//			TODO:
//		if (p->)
//		{
//			sendAllocation(p);
//		}

	}
	/**
	 * Sends an AllocationAuthorityInfo message containing the assignment of p
	 */
	void AuthorityManager::sendAllocation(RunningPlan* p)
	{
	}
	bool AuthorityManager::authorityMatchesPlan(AllocationAuthorityInfo aai, RunningPlan* p)
	{

	}

}
