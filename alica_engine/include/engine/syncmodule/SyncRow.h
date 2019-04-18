#pragma once

#include "engine/Types.h"

#include <essentials/AgentIDConstPtr.h>

#include <vector>

namespace alica
{

struct SyncData;

class SyncRow
{
public:
    SyncRow();
    SyncRow(const SyncData& sd);
    virtual ~SyncRow();
    AgentGrp& getReceivedBy();
    void setReceivedBy(const AgentGrp& recievedBy);
    const SyncData& getSyncData() const { return _syncData; }
    SyncData& editSyncData() { return _syncData; }
    void setSyncData(const SyncData& syncData);
    void toString();
    bool hasData() const { return _haveData; }
    void invalidate() { _haveData = false; }

private:
    SyncData _syncData;
    bool _haveData;
    // this vector always has to be sorted
    AgentGrp _receivedBy;
};

} /* namespace alica */
