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
#include "engine/IAlicaCommunication.h"

namespace alica
{

	SyncModul::SyncModul()
	{
		this->ae = nullptr;
		this->myId = 0;
		this->pr = nullptr;
		this->running = false;
		this->ticks = 0;
		this->communicator = nullptr;

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
		this->communicator = this->ae->getCommunicator();
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
		if (this->ae->isMaySendMessages())
			return;
		st.senderID = this->myId;
		this->communicator->sendSyncTalk(st);

	}
	void SyncModul::sendSyncReady(SyncReady sr)
	{
		if (this->ae->isMaySendMessages())
			return;
		sr.senderID = this->myId;
		communicator->sendSyncReady(sr);
	}
	void SyncModul::sendAcks(vector<SyncData> syncDataList)
	{
		if (this->ae->isMaySendMessages())
			return;
		SyncTalk st;
		st.senderID = this->myId;
		st.syncData = syncDataList;
		this->communicator->sendAcks(st);
	}
	void SyncModul::synchronisationDone(SyncTransition* st)
	{
#if SM_SUCCES
		cout << "SyncDONE in SYNCMODUL for synctransID: "  << st->getId() << endl;
#endif

		{
			lock_guard<mutex> lock(lomutex);
#if SM_SUCCES
			cout << "Remove synchronisation object for syntransID: " << st->getId() << endl;
#endif
			this->synchSet.erase(st);
		}
		this->synchedTransitions.push_back(st);
#if SM_SUCCES
				cout << "SM: SYNC TRIGGER TIME:" << this->ae->getIAlicaClock()->now()/1000000UL << endl;
#endif
	}

	bool SyncModul::followSyncTransition(Transition* trans)
	{
		list<SyncTransition*>::iterator it = find(this->synchedTransitions.begin(),this->synchedTransitions.end(),trans->getSyncTransition());
		if(it != this->synchedTransitions.end())
		{
			this->synchedTransitions.remove(trans->getSyncTransition());
			return true;
		}
		return false;
	}
	void SyncModul::onSyncTalk(SyncTalk st)
	{

	}

} /* namespace supplementary */
