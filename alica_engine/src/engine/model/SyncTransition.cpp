/*
 * SyncTransition.cpp
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/SyncTransition.h"
#include "engine/model/Plan.h"
#include "engine/model/Transition.h"

namespace alica
{

	SyncTransition::SyncTransition()
	{
		this->failOnSyncTimeOut = false;
		this->syncTimeOut = 3000;
		this->talkTimeOut = 30;
		this->plan = nullptr;
	}

	SyncTransition::~SyncTransition()
	{
	}

	string SyncTransition::toString()
	{
		stringstream ss;
		ss << "#SyncTransition: " << this->name << " " << this->id << endl;
		if(this->plan != NULL)
		{
			ss << "\t Plan: " << this->plan->getId() << " " << this->plan->getName() << endl;
		}
		ss << endl;
		ss << "\t TalkTimeOut: " << this->talkTimeOut << endl;
		ss << "\t SyncTimeOut: " << this->syncTimeOut << endl;
		ss << "\t FailOnSyncTimeOut: " << this->failOnSyncTimeOut << endl;
		ss << "\t InSync: " << this->inSync.size() << endl;
		if(this->inSync.size() != 0)
		{
			for(Transition* t : this->inSync)
			{
				ss << "\t" << t->getId() << " " << t->getName() << endl;
			}
		}
		ss << endl;
		ss << "#EndSyncTransition" << endl;
		return ss.str();

	}

	bool SyncTransition::isFailOnSyncTimeOut() const
	{
		return failOnSyncTimeOut;
	}

	void SyncTransition::setFailOnSyncTimeOut(bool failOnSyncTimeOut)
	{
		this->failOnSyncTimeOut = failOnSyncTimeOut;
	}

	unsigned long SyncTransition::getSyncTimeOut() const
	{
		return syncTimeOut;
	}

	void SyncTransition::setSyncTimeOut(unsigned long syncTimeOut)
	{
		this->syncTimeOut = syncTimeOut;
	}

	unsigned long SyncTransition::getTalkTimeOut() const
	{
		return talkTimeOut;
	}

	void SyncTransition::setTalkTimeOut(unsigned long talkTimeOut)
	{
		this->talkTimeOut = talkTimeOut;
	}

	const Plan* SyncTransition::getPlan() const
	{
		return plan;
	}

	void SyncTransition::setPlan(Plan* plan)
	{
		this->plan = plan;
	}

	list<Transition*>& SyncTransition::getInSync()
	{
		return inSync;
	}

	void SyncTransition::setInSync(const list<Transition*>& inSync)
	{
		this->inSync = inSync;
	}

} /* namespace Alica */


