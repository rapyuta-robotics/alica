/*
 * SyncRow.cpp
 *
 *  Created on: Aug 27, 2014
 *      Author: Stefan Jakob
 */

#include <algorithm>
#include <iostream>
#include <engine/containers/SyncData.h>
#include <engine/syncmodule/SyncRow.h>

namespace alica {

SyncRow::SyncRow() {
    this->syncData = nullptr;
}

SyncRow::SyncRow(SyncData* sd) {
    this->syncData = sd;
}

SyncRow::~SyncRow() {
    delete syncData;
}

vector<const supplementary::AgentID*>& SyncRow::getReceivedBy() {
    sort(this->receivedBy.begin(), this->receivedBy.end());
    return receivedBy;
}

void SyncRow::setReceivedBy(vector<const supplementary::AgentID*> receivedBy) {
    this->receivedBy = receivedBy;
}

SyncData* SyncRow::getSyncData() {
    return syncData;
}

void SyncRow::setSyncData(SyncData* syncData) {
    if (this->syncData != nullptr) {
        delete this->syncData;
    }
    this->syncData = syncData;
}
void SyncRow::toString() {
    cout << "SyncRow" << endl;
    cout << "ReceivedBy: ";
    for (auto& i : receivedBy) {
        cout << i << " ";
    }
    cout << endl;
    this->syncData->toString();
}

} /* namespace alica */
