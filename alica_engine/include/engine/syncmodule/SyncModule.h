#pragma once

#include "supplementary/IAgentID.h"

#include <list>
#include <mutex>
#include <iostream>
#include <map>
#include <vector>
#include <memory>
//#define SM_SUCCES

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

	class SyncModule
	{
	public:
		SyncModule(AlicaEngine* ae);
		virtual ~SyncModule();
		virtual void init();
		virtual void close();
		virtual void tick();
		virtual void setSynchronisation(Transition* trans, bool holds);
		virtual bool followSyncTransition(Transition* trans);
		virtual void onSyncTalk(std::shared_ptr<SyncTalk> st);
		virtual void onSyncReady(std::shared_ptr<SyncReady> sr);

		void sendSyncTalk(SyncTalk& st);
		void sendSyncReady(SyncReady& sr);
		void sendAcks(std::vector<SyncData*> syncDataList);
		void synchronisationDone(SyncTransition* st);
	protected:
		bool running;
		AlicaEngine* ae;
		const supplementary::IAgentID* myId;
		unsigned long ticks;
		PlanRepository* pr;
		std::map<SyncTransition*, Synchronisation*> synchSet;
		std::list<SyncTransition*> synchedTransitions;
		std::mutex lomutex;
		const IAlicaCommunication* communicator;

	};

} /* namespace supplementary */
