/*
 * Synchronisation.h
 *
 *  Created on: Aug 27, 2014
 *      Author: Stefan Jakob
 */

#ifndef SYNCHRONISATION_H_
#define SYNCHRONISATION_H_
#define SM_MISC

using namespace std;

#include <list>

namespace alica
{

	class SyncModul;
	class SyncTransition;
	class SyncRow;
	class RunningPlan;
	struct SyncData;
	struct SyncReady;
	struct SyncTalk;


	class Synchronisation
	{
	public:
		Synchronisation();
		Synchronisation(int myID, SyncTransition* st, SyncModul* sm);
		virtual ~Synchronisation();
		void setTick(unsigned long now);
		void changeOwnData (long transitionID, bool conditionHolds);
		bool IsValid(unsigned long curTick);
		bool integrateSyncTalk(SyncTalk* talk, unsigned long curTick);
		void integrateSyncReady(SyncReady* ready);
		SyncTransition* getSyncTransition();
		void setSyncTransition(SyncTransition* syncTransition);

	private:
		bool allSyncReady();
		void printMatrix();

	protected:
		SyncModul* syncModul;
		SyncTransition* syncTransition;
		int myID;
		unsigned long lastTalkTime;
		SyncData* lastTalkData;
		unsigned long syncStartTime;
		bool readyForSync;
		unsigned long lastTick;
		list<SyncReady*> receivedSyncReadys;
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
