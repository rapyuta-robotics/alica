/*
 * SyncRow.cpp
 *
 *  Created on: Aug 27, 2014
 *      Author: Stefan Jakob
 */

#include <algorithm>
#include <engine/containers/SyncData.h>
#include <engine/syncmodule/SyncRow.h>
#include <iostream>

namespace alica
{

SyncRow::SyncRow()
    : _haveData(false)
{
}

SyncRow::SyncRow(const SyncData& sd)
    : _syncData(sd)
    , _haveData(true)
{
}

SyncRow::~SyncRow() {}

std::vector<const supplementary::AgentID*>& SyncRow::getReceivedBy()
{
    sort(_receivedBy.begin(), _receivedBy.end(), supplementary::AgentIDComparator());
    return _receivedBy;
}

void SyncRow::setReceivedBy(std::vector<const supplementary::AgentID*> receivedBy)
{
    _receivedBy = receivedBy;
}

void SyncRow::setSyncData(const SyncData& syncData)
{
    _syncData = syncData;
}
void SyncRow::toString()
{ // TODO: fix this method (doesnt produce a string, but write to cout)
    std::cout << "SyncRow" << std::endl;
    std::cout << "ReceivedBy: ";
    for (auto& i : _receivedBy) {
        std::cout << i << " ";
    }
    std::cout << std::endl;
    _syncData.toString();
}

} /* namespace alica */
