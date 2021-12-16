#pragma once

#include "engine/Types.h"

#include <vector>

namespace alica
{

struct SyncData;

class SyncRow
{
public:
    SyncRow() = delete;
    explicit SyncRow(const SyncData& sd);
    virtual ~SyncRow();

    const AgentGrp& getReceivedBy();
    AgentGrp& editReceivedBy();

    const SyncData& getSyncData() const { return _syncData; }
    SyncData& editSyncData() { return _syncData; }
    void setSyncData(const SyncData& syncData);

    friend std::ostream& operator<<(std::ostream& os, const SyncRow& sr);

private:
    SyncData _syncData;
    AgentGrp _receivedBy; /**< this vector always has to be sorted */
};
} /* namespace alica */
