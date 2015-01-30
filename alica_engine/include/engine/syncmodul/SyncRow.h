/*
 * SyncRow.h
 *
 *  Created on: Aug 27, 2014
 *      Author: Stefan Jakob
 */

#ifndef SYNCROW_H_
#define SYNCROW_H_

using namespace std;

#include <vector>

namespace alica
{

	struct SyncData;

	class SyncRow
	{
	public:
		SyncRow();
		SyncRow(SyncData* sd);
		virtual ~SyncRow();
		vector<int>& getReceivedBy();
		void setReceivedBy(vector<int> recievedBy);
		SyncData* getSyncData();
		void setSyncData(SyncData* syncData);
		void toString();


	protected:
		SyncData* syncData;
		//this vector always has to be sorted
		vector<int> receivedBy;
	};

} /* namespace alica */

#endif /* SYNCROW_H_ */
