/*
 * SyncRow.cpp
 *
 *  Created on: Aug 27, 2014
 *      Author: Stefan Jakob
 */

#include <engine/syncmodul/SyncRow.h>

namespace alica
{

	SyncRow::SyncRow()
	{
		// TODO Auto-generated constructor stub

	}

	SyncRow::SyncRow(SyncData* sd)
	{
	}

	SyncRow::~SyncRow()
	{
		// TODO Auto-generated destructor stub
	}

	vector<int> SyncRow::getRecievedBy()
	{
		return recievedBy;
	}

	void SyncRow::setRecievedBy(vector<int> recievedBy)
	{
		this->recievedBy = recievedBy;
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
