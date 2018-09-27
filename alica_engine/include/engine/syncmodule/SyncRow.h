#pragma once

#include "supplementary/AgentID.h"

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
    std::vector<const supplementary::AgentID*>& getReceivedBy();
    void setReceivedBy(std::vector<const supplementary::AgentID*> recievedBy);
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
    std::vector<const supplementary::AgentID*> _receivedBy;
};

} /* namespace alica */
