/*
 * SyncTransition.cpp
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/SyncTransition.h"

namespace alica
{

	SyncTransition::SyncTransition()
	{
		this->failOnSyncTimeOut = false;
		this->syncTimeOut = 3000;
		this->talkTimeOut = 30;
	}

	SyncTransition::~SyncTransition()
	{
		// TODO Auto-generated destructor stub
	}

	bool alica::SyncTransition::isFailOnSyncTimeOut() const
	{
		return failOnSyncTimeOut;
	}

	void alica::SyncTransition::setFailOnSyncTimeOut(bool failOnSyncTimeOut)
	{
		this->failOnSyncTimeOut = failOnSyncTimeOut;
	}

	unsigned long alica::SyncTransition::getSyncTimeOut() const
	{
		return syncTimeOut;
	}

	void alica::SyncTransition::setSyncTimeOut(unsigned long syncTimeOut)
	{
		this->syncTimeOut = syncTimeOut;
	}

	unsigned long alica::SyncTransition::getTalkTimeOut() const
	{
		return talkTimeOut;
	}

	void alica::SyncTransition::setTalkTimeOut(unsigned long talkTimeOut)
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

} /* namespace Alica */
