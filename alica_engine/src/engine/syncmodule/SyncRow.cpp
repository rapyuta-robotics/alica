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
{
    this->syncData = nullptr;
}

SyncRow::SyncRow(SyncData* sd)
{
    this->syncData = sd;
}

SyncRow::~SyncRow()
{
    delete syncData;
}

std::vector<const supplementary::AgentID*>& SyncRow::getReceivedBy()
{
    sort(this->receivedBy.begin(), this->receivedBy.end(), supplementary::AgentIDComparator());
    return receivedBy;
}

void SyncRow::setReceivedBy(std::vector<const supplementary::AgentID*> receivedBy)
{
    this->receivedBy = receivedBy;
}

SyncData* SyncRow::getSyncData()
{
    return syncData;
}

void SyncRow::setSyncData(SyncData* syncData)
{
    if (this->syncData != nullptr) {
        delete this->syncData;
    }
    this->syncData = syncData;
}
void SyncRow::toString()
{ // TODO: fix this method (doesnt produce a string, but write to cout)
    std::cout << "SyncRow" << std::endl;
    std::cout << "ReceivedBy: ";
    for (auto& i : receivedBy) {
        std::cout << i << " ";
    }
    std::cout << std::endl;
    this->syncData->toString();
}

} /* namespace alica */
