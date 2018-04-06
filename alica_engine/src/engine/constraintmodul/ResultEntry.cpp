/*
 * ResultEntry.cpp
 *
 *  Created on: Nov 24, 2014
 *      Author: Philipp
 */
#include "engine/constraintmodul/ResultEntry.h"

#include <engine/model/Variable.h>
#include <engine/AlicaEngine.h>
#include <engine/AlicaClock.h>
#include <engine/containers/SolverVar.h>
#include <engine/containers/SolverResult.h>
#include <limits>

using namespace std;

namespace alica {

ResultEntry::ResultEntry(const supplementary::AgentID* robotId, const AlicaEngine* ae) {
    this->ae = ae;
    this->id = robotId;
}

ResultEntry::~ResultEntry() {
    // TODO Auto-generated destructor stub
}

const supplementary::AgentID* ResultEntry::getId() {
    return id;
}

void ResultEntry::addValue(long vid, shared_ptr<vector<uint8_t>> val) {
    long now = ae->getAlicaClock()->now();
    shared_ptr<VarValue> vv;
    lock_guard<std::mutex> lock(valueLock);
    auto it = this->values.find(vid);
    if (it != values.end()) {
        vv = it->second;
        vv->val = val;
        vv->lastUpdate = now;
    } else {
        vv = make_shared<VarValue>(vid, val, now);
        this->values[vid] = vv;
    }
}

void ResultEntry::clear() {
    lock_guard<std::mutex> lock(valueLock);
    this->values.clear();
}

shared_ptr<vector<SolverVar*>> ResultEntry::getCommunicatableResults(long ttl4Communication) {
    lock_guard<std::mutex> lock(valueLock);
    shared_ptr<vector<SolverVar*>> lv = make_shared<vector<SolverVar*>>();
    long now = ae->getAlicaClock()->now();
    for (auto iterator = values.begin(); iterator != values.end(); iterator++) {
        if (iterator->second->lastUpdate + ttl4Communication > now) {
            SolverVar* sv = new SolverVar();
            sv->id = iterator->second->id;
            sv->value = *iterator->second->val;
            lv->push_back(sv);
        }
    }
    return lv;
}

shared_ptr<vector<uint8_t>> ResultEntry::getValue(long vid, long ttl4Usage) {
    long now = ae->getAlicaClock()->now();
    lock_guard<std::mutex> lock(valueLock);
    auto it = this->values.find(vid);
    if (it != values.end()) {
        if (it->second->lastUpdate + ttl4Usage > now) {
            return it->second->val;
        }
    }
    return nullptr;
}

shared_ptr<vector<shared_ptr<vector<uint8_t>>>> ResultEntry::getValues(
        shared_ptr<vector<Variable*>> query, long ttl4Usage) {
    shared_ptr<vector<shared_ptr<vector<uint8_t>>>> ret =
            make_shared<vector<shared_ptr<vector<uint8_t>>>>(query->size());
    int i = 0;
    int nans = 0;
    for (auto it = query->begin(); it != query->end(); it++, i++) {
        ret->at(i) = getValue((*it)->getId(), ttl4Usage);
        if (ret->at(i) == nullptr)
            nans++;
    }
    if (nans == i)
        return nullptr;
    return ret;
}

} /* namespace alica */
