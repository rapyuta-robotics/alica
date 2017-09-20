/*
 * SyncRow.h
 *
 *  Created on: Aug 27, 2014
 *      Author: Stefan Jakob
 */

#ifndef SYNCROW_H_
#define SYNCROW_H_

#include "engine/IRobotID.h"

#include <vector>

using namespace std;
namespace alica
{

	struct SyncData;

	class SyncRow
	{
	public:
		SyncRow();
		SyncRow(SyncData* sd);
		virtual ~SyncRow();
		vector<alica::IRobotID>& getReceivedBy();
		void setReceivedBy(vector<alica::IRobotID> recievedBy);
		SyncData* getSyncData();
		void setSyncData(SyncData* syncData);
		void toString();


	protected:
		SyncData* syncData;
		//this vector always has to be sorted
		vector<alica::IRobotID> receivedBy;
	};

} /* namespace alica */

#endif /* SYNCROW_H_ */
