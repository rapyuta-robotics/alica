/*
 * SyncModul.cpp
 *
 *  Created on: Aug 27, 2014
 *      Author: Paul Panin
 */

#include "engine/syncmodul/SyncModul.h"
#include "engine/AlicaEngine.h"
#include "engine/teamobserver/TeamObserver.h"
#include "engine/PlanRepository.h"
#include "engine/syncmodul/Synchronisation.h"
#include "engine/model/Transition.h"
#include "engine/containers/SyncData.h"
#include "engine/containers/SyncReady.h"
#include "engine/containers/SyncTalk.h"

namespace alica
{

	SyncModul::SyncModul()
	{
		this->ae = nullptr;
		this->myId = 0;
		this->pr = nullptr;
		this->running = false;
		this->syncReadyPublisher = nullptr;
		this->syncTalkPublisher = nullptr;
		this->ticks = 0;

	}

	SyncModul::~SyncModul()
	{
		// TODO Auto-generated destructor stub
	}

	void SyncModul::init()
	{
		this->ticks = 0;
		this->running = true;
		this->ae = AlicaEngine::getInstance();
		this->myId = ae->getTeamObserver()->getOwnId();
		this->pr = this->ae->getPlanRepository();
		//TODO
//		syncTalkPublisher = this.rosNode.Advertise("SyncTalk", SyncTalk.TypeId, 5);
//		syncReadyPublisher = this.rosNode.Advertise("SyncReady", SyncReady.TypeId, 5);
//
//		this.rosNode.Subscribe("SyncTalk", OnSyncTalk, 10);
//		this.rosNode.Subscribe("SyncReady", OnSyncReady, 10);

	}
	void SyncModul::close()
	{
		this->running = false;
		cout << "SynchModul: Closed SynchModul" << endl;
	}
	void SyncModul::tick()
	{
		list<Synchronisation*> failedSyncs;
		lock_guard<mutex> lock(lomutex);
		for (auto iter : this->synchSet)
		{
			if (!iter.second->isValid(ticks))
			{
				failedSyncs.push_back(iter.second);
			}
			ticks++;
			for (Synchronisation* s : failedSyncs)
			{
				this->synchSet.erase(s->getSyncTransition());
			}
		}

	}
	void SyncModul::setSynchronisation(Transition* trans, bool holds)
	{
		Synchronisation* s;
		map<SyncTransition*, Synchronisation*>::iterator i = this->synchSet.find(trans->getSyncTransition());
		if (i != this->synchSet.end())
		{
			i->second->setTick(this->ticks);
			i->second->changeOwnData(trans->getId(), holds);
		}
		else
		{
			s = new Synchronisation(myId, trans->getSyncTransition(), this);
			s->setTick(this->ticks);
			s->changeOwnData(trans->getId(), holds);
			{
				lock_guard<mutex> lock(lomutex);
				synchSet.insert(pair<SyncTransition*, Synchronisation*>(trans->getSyncTransition(), s));
			}
		}
	}
	void SyncModul::sendSyncTalk(SyncTalk st)
	{
//		if(this->ae->)
//			return;
//		st.senderID = this->myId;

	}
	void SyncModul::sendSyncReady(SyncReady sr)
	{

	}
	void SyncModul::sendAcks(list<SyncData> syncDataList)
	{

	}

	bool SyncModul::followSyncTransition(Transition* trans)
	{

	}

} /* namespace supplementary */
