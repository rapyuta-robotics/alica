/*
 * ResultEntry.cpp
 *
 *  Created on: Nov 24, 2014
 *      Author: Philipp
 */
#include "engine/constraintmodul/ResultEntry.h"

#include <engine/model/Variable.h>
#include <engine/AlicaEngine.h>
#include <engine/containers/SolverVar.h>
#include <engine/containers/SolverResult.h>
#include <limits>

using namespace std;

namespace alica {

ResultEntry::ResultEntry(const supplementary::AgentID* robotId) 
    : _id(robotId)
    {}



void ResultEntry::addValue(int64_t vid, Variant val, AlicaTime time) {
    lock_guard<std::mutex> lock(_valueLock);
    auto it = _values.find(vid);
    if (it != values.end()) {
        VarValue& vv = it->second;
        vv->_val = val;
        vv->_lastUpdate = time;
    } else {
        _values[vid] = VarValue(vid, val, time);
    }
}

void ResultEntry::clear() {
    lock_guard<std::mutex> lock(_valueLock);
    _values.clear();
}

void ResultEntry::getCommunicatableResults(AlicaTime earliest, std::vector<SolverVar>& o_result) {
    lock_guard<std::mutex> lock(_valueLock);
    for (const std::pair<int64_t, VarValue>& p : _values) {
        if (p.second.lastUpdate > earliest) {
            SolverVar sv;
            sv.id = p.second.id;
            p.second.serializeTo(sv.value);
            o_result.push_back(std::move(sv));
        }
    }
}

Variant ResultEntry::getValue(int64_t vid, AlicaTime earliest) {
    long now = ae->getIAlicaClock()->now();
    lock_guard<std::mutex> lock(valueLock);
    auto it = this->values.find(vid);
    if (it != values.end()) {
        if (it->second.lastUpdate > earliest) {
            return it->second._val;
        }
    }
    return Variant();
}

bool ResultEntry::getValues(const VariableSet& query, AlicaTime earliest, std::vector<Variant>& o_values) {
    o_values.resize(query->size());
    int i = 0;
    int nans = 0;
    for (const Variable* v : query) {
        o_values[i] = getValue(v->getId(), earliest);
        if (!o_values[i].isSet()) {
            ++nans;
        }
        ++i;
    }
    return nans != i;
}

} /* namespace alica */
