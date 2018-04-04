/*
 * SuccessCollection.cpp
 *
 *  Created on: Jun 17, 2014
 *      Author: Stefan Jakob
 */

#include <engine/collections/SuccessCollection.h>

#include "engine/model/Plan.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Task.h"

namespace alica {

SuccessCollection::SuccessCollection(const Plan* plan) {
    this->count = plan->getEntryPoints().size();
    this->entryPoints = new EntryPoint*[this->count];
    this->robotIds = vector<shared_ptr<list<const supplementary::AgentID*>>>(this->count);
    int i = 0;
    
    for (map<long, const EntryPoint*>::const_iterator iter = plan->getEntryPoints().begin();
            iter != plan->getEntryPoints().end(); ++iter) {
        eps[i] =iter->second;
        
        ++i;
    }
    std::sort(entryPoints,entryPoints+count,EntryPoint::compareTo);
    for(int j=0; j<count; ++j) {
        this->robotIds[j] = make_shared<list<const supplementary::AgentID*>>();
    }
}

SuccessCollection::~SuccessCollection() {
    delete[] this->entryPoints;
}

int SuccessCollection::getCount() const {
    return count;
}

void SuccessCollection::setCount(int count) {
    this->count = count;
}

const EntryPoint** SuccessCollection::getEntryPoints() {
    return entryPoints;
}

void SuccessCollection::setSuccess(const supplementary::AgentID* robotId, const EntryPoint* ep) {
    for (int i = 0; i < this->count; i++) {
        if (this->entryPoints[i] == ep) {
            this->robotIds[i]->push_back(robotId);
            return;
        }
    }
}
void SuccessCollection::clear() {
    for (int i = 0; i < this->count; i++) {
        this->robotIds[i]->clear();
    }
}

vector<shared_ptr<list<const supplementary::AgentID*>>>& SuccessCollection::getRobots() {
    return robotIds;
}

void SuccessCollection::setRobots(vector<shared_ptr<list<const supplementary::AgentID*>>>& robotIds) {
    this->robotIds = robotIds;
}

std::shared_ptr<std::list<const supplementary::AgentID*>> SuccessCollection::getRobots(const EntryPoint* ep) {
    for (int i = 0; i < this->count; i++) {
        if (this->getEntryPoints()[i] == ep) {
            return this->robotIds[i];
        }
    }
    return nullptr;
}

std::shared_ptr<std::list<const supplementary::AgentID*>> SuccessCollection::getRobotsById(int64_t id) {
    for (int i = 0; i < this->count; i++) {
        if (this->getEntryPoints()[i]->getId() == id) {
            return this->robotIds[i];
        }
    }
    return nullptr;
}

std::string SuccessCollection::toString() const {
    std::stringstream ss;
    ss << "";
    for (int i = 0; i < this->count; i++) {
        if (this->robotIds[i]->size() > 0) {
            ss << this->entryPoints[i]->getTask()->getId() << ": ";
            for (const supplementary::AgentID* r : *(this->robotIds[i])) {
                ss << *r << " ";
            }
            ss << endl;
        }
    }
    if (ss.str().compare("") != 0) {
        return "Successes: \n" + ss.str();
    }
    return "No EntryPoint successful!";
}

} /* namespace alica */
