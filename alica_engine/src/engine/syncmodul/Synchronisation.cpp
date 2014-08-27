/*
 * Synchronisation.cpp
 *
 *  Created on: Aug 27, 2014
 *      Author: Stefan Jakob
 */

#include <engine/syncmodul/Synchronisation.h>

namespace alica
{

	Synchronisation::Synchronisation()
	{
		// TODO Auto-generated constructor stub

	}

	Synchronisation::Synchronisation(int myID, SyncTransition* st, SyncModul* sm)
	{
	}

	Synchronisation::~Synchronisation()
	{
		// TODO Auto-generated destructor stub
	}

	SyncTransition* Synchronisation::getSyncTransition()
	{
		return syncTransition;
	}

	void Synchronisation::setTick(unsigned long now)
	{
	}

	void Synchronisation::changeOwnData(long transitionID, bool conditionHolds)
	{
	}

	bool Synchronisation::IsValid(unsigned long curTick)
	{
	}

	bool Synchronisation::integrateSyncTalk(SyncTalk* talk, unsigned long curTick)
	{
	}

	void Synchronisation::integrateSyncReady(SyncReady* ready)
	{
	}

	void Synchronisation::setSyncTransition(SyncTransition* syncTransition)
	{
		this->syncTransition = syncTransition;
	}

	bool Synchronisation::allSyncReady()
	{
	}

	void Synchronisation::printMatrix()
	{
	}

	void Synchronisation::sendTalk(SyncData* sd)
	{
	}

	void Synchronisation::sendSyncReady()
	{
	}

	bool Synchronisation::isSyncComplete()
	{
	}

} /* namespace alica */
