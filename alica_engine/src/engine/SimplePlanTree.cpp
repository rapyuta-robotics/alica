/*
 * SimplePlanTree.cpp
 *
 *  Created on: Jun 16, 2014
 *      Author: Stefan Jakob
 */

#include <engine/SimplePlanTree.h>
#include "engine/model/EntryPoint.h"
#include "engine/model/Plan.h"
#include "engine/model/Task.h"
#include "engine/model/State.h"

namespace alica {

SimplePlanTree::SimplePlanTree()
        : robotId(nullptr) {
    this->state = nullptr;
    this->newSimplePlanTree = true;
    this->receiveTime = 0;
    this->entryPoint = nullptr;
    this->parent = nullptr;
}

SimplePlanTree::~SimplePlanTree() {}

bool SimplePlanTree::containsPlan(const AbstractPlan* plan) const {
    if (this->getEntryPoint()->getPlan() == plan) {
        return true;
    }
    for (const std::shared_ptr<SimplePlanTree>& spt : this->getChildren()) {
        if (spt->containsPlan(plan)) {
            return true;
        }
    }
    return false;
}


void SimplePlanTree::setEntryPoint(const EntryPoint* entryPoint) {
    this->entryPoint = entryPoint;
}

void SimplePlanTree::setState(const State* state) {
    this->state = state;
}

const std::unordered_set<std::shared_ptr<SimplePlanTree>>& SimplePlanTree::getChildren() const {
    return children;
}

void SimplePlanTree::setChildren(const std::unordered_set<std::shared_ptr<SimplePlanTree>>& children) {
    this->children = children;
}

const supplementary::AgentID* SimplePlanTree::getRobotId() {
    return robotId;
}

void SimplePlanTree::setRobotId(const supplementary::AgentID* robotId) {
    this->robotId = robotId;
}

bool SimplePlanTree::isNewSimplePlanTree() const {
    return newSimplePlanTree;
}

void SimplePlanTree::setNewSimplePlanTree(bool newSimplePlanTree) {
    this->newSimplePlanTree = newSimplePlanTree;
}
long SimplePlanTree::getReceiveTime() const {
    return receiveTime;
}

void SimplePlanTree::setReceiveTime(long receiveTime) {
    this->receiveTime = receiveTime;
}

const std::list<int64_t>& SimplePlanTree::getStateIds() const {
    return stateIds;
}

void SimplePlanTree::setStateIds(const std::list<int64_t>& stateIds) {
    this->stateIds = stateIds;
}

std::string SimplePlanTree::toString() const {
    std::stringstream result;

    result << "RobotID: " << this->robotId << "\n";
    result << "Parent: ";

    if (this->parent != nullptr) {
        result << parent->getState()->getId();
    }
    result << "\n";

    result << "State: ";
    if (state != nullptr) {
        result << state->getId();
        result << " " + state->getName();
    } else {
        result << "ERROR !!!NO STATE !!!";
    }

    result << "\n";

    result << "EntryPoint: ";

    if (this->entryPoint != nullptr) {
        result << entryPoint->getId() << " " << this->entryPoint->getTask()->getName();
    } else {
        result << "NoEntryPoint";
    }
    result << "\n";

    result << "Children: " << this->children.size() << "\n";
    for (auto spt : this->children) {
        result << spt->toString();
    }

    result << "\n\n";

    return result.str();
}

} /* namespace alica */
