/*
 * Plan.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/Plan.h"
#include "engine/model/Task.h"
#include "engine/model/EntryPoint.h"

namespace alica {
Plan::Plan(long id) : AbstractPlan() {
    this->postCondition = nullptr;
    this->id = id;
    this->minCardinality = 0;
    this->maxCardinality = 0;
}

Plan::~Plan() {}

string Plan::toString() const {
    stringstream ss;
    ss << AbstractPlan::toString();
    ss << "Filename: " << this->fileName << endl;
    return ss.str();
}

EntryPoint* Plan::getEntryPointTaskID(long taskID) {
    for (map<long, alica::EntryPoint*>::const_iterator iter = entryPoints.begin(); iter != entryPoints.end(); iter++) {
        const Task* task = iter->second->getTask();
        if (task != nullptr) {
            if (task->getId() == taskID) {
                return iter->second;
            }
        } else {
            cout << "Model: Class Plan: Entrypoint with ID " << iter->second->getId() << " does not have a Task"
                 << endl;
            throw new exception();
        }
    }
    return nullptr;
}

//===================== Getter and Setter ==================

const string& Plan::getFileName() const {
    if (this->fileName.empty()) {
        static string result = this->name + ".pml";
        return result;
    } else {
        return this->fileName;
    }
}

map<long, EntryPoint*>& Plan::getEntryPoints() {
    return entryPoints;
}

void Plan::setEntryPoints(const map<long, EntryPoint*>& entryPoints) {
    this->entryPoints = entryPoints;
}

list<FailureState*>& Plan::getFailureStates() {
    return failureStates;
}

void Plan::setFailureStates(const list<FailureState*>& failureStates) {
    this->failureStates = failureStates;
}

int Plan::getMaxCardinality() {
    return this->maxCardinality;
}

void Plan::setMaxCardinality(int maxCardinality) {
    this->maxCardinality = maxCardinality;
}

int Plan::getMinCardinality() {
    return this->minCardinality;
}

void Plan::setMinCardinality(int minCardinality) {
    this->minCardinality = minCardinality;
}

PostCondition* Plan::getPostCondition() {
    return postCondition;
}

void Plan::setPostCondition(PostCondition* postCondition) {
    this->postCondition = postCondition;
}

list<State*>& Plan::getStates() {
    return states;
}

void Plan::setStates(const list<State*>& states) {
    this->states = states;
}

list<SuccessState*>& Plan::getSuccessStates() {
    return successStates;
}

void Plan::setSuccessStates(const list<SuccessState*>& successStates) {
    this->successStates = successStates;
}

list<SyncTransition*>& Plan::getSyncTransitions() {
    return syncTransitions;
}

void Plan::setSyncTransitions(const list<SyncTransition*>& syncTransitions) {
    this->syncTransitions = syncTransitions;
}

list<Transition*>& Plan::getTransitions() {
    return transitions;
}

void Plan::setTransitions(const list<Transition*>& transitions) {
    this->transitions = transitions;
}

string alica::Plan::getDestinationPath() {
    return destinationPath;
}

void alica::Plan::setDestinationPath(string destinationPath) {
    this->destinationPath = destinationPath;
}

}  // namespace alica
