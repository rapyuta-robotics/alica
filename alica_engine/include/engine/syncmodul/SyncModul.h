/*
 * SyncModul.h
 *
 *  Created on: Aug 27, 2014
 *      Author: Paul Panin
 */

#ifndef SYNCMODUL_H_
#define SYNCMODUL_H_

#include <list>
#include "../ISyncModul.h"
#include <mutex>
#include <iostream>
#include <map>
#include <vector>
//#define SM_SUCCES

using namespace std;

namespace alica
{
	class Transition;
	class SyncTransition;
	class AlicaEngine;
	class PlanRepository;
	class Synchronisation;
	struct SyncData;
	struct SyncReady;
	struct SyncTalk;
	class IAlicaCommunication;

	class SyncModul : public ISyncModul
	{
	public:
		SyncModul(AlicaEngine* ae);
		virtual ~SyncModul();
		virtual void init();
		virtual void close();
		virtual void tick();
		virtual void setSynchronisation(Transition* trans, bool holds);
		virtual bool followSyncTransition(Transition* trans);
		virtual void onSyncTalk(shared_ptr<SyncTalk> st);
		virtual void onSyncReady(shared_ptr<SyncReady> sr);

		void sendSyncTalk(SyncTalk& st);
		void sendSyncReady(SyncReady& sr);
		void sendAcks(vector<SyncData*> syncDataList);
		void synchronisationDone(SyncTransition* st);
	protected:
		bool running;
		AlicaEngine* ae;
		int myId;
		unsigned long ticks;
		PlanRepository* pr;
		map<SyncTransition*, Synchronisation*> synchSet;
		list<SyncTransition*> synchedTransitions;
		mutex lomutex;
		IAlicaCommunication* communicator;

	};

} /* namespace supplementary */

#endif /* SYNCMODUL_H_ */
