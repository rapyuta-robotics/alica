#include <engine/containers/SyncData.h>
#include <engine/syncmodule/SyncRow.h>

#include <algorithm>
#include <iostream>

namespace alica
{

SyncRow::SyncRow(const SyncData& sd)
        : _syncData(sd)
{
}

SyncRow::~SyncRow() {}

const AgentGrp& SyncRow::getReceivedBy()
{
    sort(_receivedBy.begin(), _receivedBy.end());
    return _receivedBy;
}

AgentGrp& SyncRow::editReceivedBy()
{
    sort(_receivedBy.begin(), _receivedBy.end());
    return _receivedBy;
}

void SyncRow::setSyncData(const SyncData& syncData)
{
    _syncData = syncData;
}

std::ostream& operator<<(std::ostream& os, const SyncRow& sr)
{
    std::stringstream ss;
    ss << "## SyncRow: ReceivedBy: ";
    for (uint64_t i : sr._receivedBy) {
        ss << i << " ";
    }
    ss << std::endl << sr._syncData << "##" << std::endl;
    return os << ss.str();
}

} /* namespace alica */
