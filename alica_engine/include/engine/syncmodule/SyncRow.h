#pragma once

#include "supplementary/AgentID.h"

#include <vector>

using namespace std;
namespace alica {

struct SyncData;

class SyncRow {
public:
    SyncRow();
    SyncRow(SyncData* sd);
    virtual ~SyncRow();
    vector<const supplementary::AgentID*>& getReceivedBy();
    void setReceivedBy(vector<const supplementary::AgentID*> recievedBy);
    SyncData* getSyncData();
    void setSyncData(SyncData* syncData);
    void toString();

protected:
    SyncData* syncData;
    // this vector always has to be sorted
    vector<const supplementary::AgentID*> receivedBy;
};

} /* namespace alica */
