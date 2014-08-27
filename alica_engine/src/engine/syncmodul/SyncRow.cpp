/*
 * SyncRow.cpp
 *
 *  Created on: Aug 27, 2014
 *      Author: Stefan Jakob
 */

#include <engine/syncmodul/SyncRow.h>
#include <algorithm>

namespace alica
{

	SyncRow::SyncRow()
	{
		this->syncData = nullptr;
	}

	SyncRow::SyncRow(SyncData* sd)
	{
		this->syncData = sd;
	}

	SyncRow::~SyncRow()
	{
	}

	vector<int> SyncRow::getReceivedBy()
	{
		sort(this->receivedBy.begin(), this->receivedBy.end());
		return receivedBy;
	}

	void SyncRow::setReceivedBy(vector<int> receivedBy)
	{
		this->receivedBy = receivedBy;
	}

	SyncData* SyncRow::getSyncData()
	{
		return syncData;
	}

	void SyncRow::setSyncData(SyncData* syncData)
	{
		this->syncData = syncData;
	}

} /* namespace alica */
