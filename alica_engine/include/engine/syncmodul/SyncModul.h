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

	class SyncModul : public ISyncModul
	{
	public:
		SyncModul();
		virtual ~SyncModul();
		virtual void init();
		virtual void close();
		virtual void tick();
		virtual void setSynchronisation(Transition* trans, bool holds);
		virtual bool followSyncTransition(Transition* trans);
		void sendSyncTalk (SyncTalk st);
		void sendSyncReady(SyncReady sr);
		void sendAcks(list<SyncData> syncDataList);

	protected:
		bool running;
		AlicaEngine* ae;
		int* syncTalkPublisher;
		int* syncReadyPublisher;
		int myId;
		unsigned long ticks;
		PlanRepository* pr;
		map<SyncTransition*, Synchronisation*> synchSet;
		list<SyncTransition*> synchedTransitions;
		mutex lomutex;

	};

} /* namespace supplementary */

#endif /* SYNCMODUL_H_ */
