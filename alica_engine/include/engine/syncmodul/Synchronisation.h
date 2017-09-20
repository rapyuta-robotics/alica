/*
 * Synchronisation.h
 *
 *  Created on: Aug 27, 2014
 *      Author: Stefan Jakob
 */

#ifndef SYNCHRONISATION_H_
#define SYNCHRONISATION_H_
//#define SM_MISC
//#define SM_SUCCESS
//#define SM_FAILURE
//#define SM_MESSAGES

#include "engine/IRobotID.h"
#include <list>
#include <mutex>
#include <memory>

using namespace std;

namespace alica
{

	class SyncModul;
	class SyncTransition;
	class SyncRow;
	class RunningPlan;
	struct SyncData;
	struct SyncReady;
	struct SyncTalk;
	class AlicaEngine;


	class Synchronisation
	{
	public:
		Synchronisation(AlicaEngine* ae);
		Synchronisation(AlicaEngine* ae, alica::IRobotID myID, SyncTransition* st, SyncModul* sm);
		virtual ~Synchronisation();
		void setTick(unsigned long now);
		void changeOwnData (long transitionID, bool conditionHolds);
		bool isValid(unsigned long curTick);
		bool integrateSyncTalk(shared_ptr<SyncTalk> talk, unsigned long curTick);
		void integrateSyncReady(shared_ptr<SyncReady> ready);
		SyncTransition* getSyncTransition();
		void setSyncTransition(SyncTransition* syncTransition);

	private:
		bool allSyncReady();
		void printMatrix();

	protected:
		AlicaEngine* ae;
		mutex syncMutex;
		mutex rowOkMutex;
		SyncModul* syncModul;
		SyncTransition* syncTransition;
		alica::IRobotID myID;
		unsigned long lastTalkTime;
		SyncData* lastTalkData;
		unsigned long syncStartTime;
		bool readyForSync;
		unsigned long lastTick;
		list<shared_ptr<SyncReady>> receivedSyncReadys;
		list<long> connectedTransitions;
		RunningPlan* runningPlan;
		list<SyncRow*> rowsOK;
		SyncRow* myRow;
		list<SyncRow*> syncMatrix;
		void sendTalk(SyncData* sd);
		void sendSyncReady();
		bool isSyncComplete();

	};

} /* namespace alica */

#endif /* SYNCHRONISATION_H_ */
