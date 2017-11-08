#pragma once

#include "supplementary/IAgentID.h"

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
		vector<const supplementary::IAgentID*>& getReceivedBy();
		void setReceivedBy(vector<const supplementary::IAgentID*> recievedBy);
		SyncData* getSyncData();
		void setSyncData(SyncData* syncData);
		void toString();


	protected:
		SyncData* syncData;
		//this vector always has to be sorted
		vector<const supplementary::IAgentID*> receivedBy;
	};

} /* namespace alica */
