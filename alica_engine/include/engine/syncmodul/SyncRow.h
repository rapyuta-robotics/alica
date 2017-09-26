#pragma once

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
		vector<const alica::IRobotID*>& getReceivedBy();
		void setReceivedBy(vector<const alica::IRobotID*> recievedBy);
		SyncData* getSyncData();
		void setSyncData(SyncData* syncData);
		void toString();


	protected:
		SyncData* syncData;
		//this vector always has to be sorted
		vector<const alica::IRobotID*> receivedBy;
	};

} /* namespace alica */
