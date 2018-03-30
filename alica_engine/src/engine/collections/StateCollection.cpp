#include "engine/collections/StateCollection.h"
#include "engine/model/State.h"
#include "engine/model/EntryPoint.h"
#include "engine/collections/AssignmentCollection.h"
#include "engine/Assignment.h"

namespace alica {

StateCollection::StateCollection() {}

StateCollection::~StateCollection() {}

vector<const supplementary::AgentID*>& StateCollection::getRobots() {
    return robotIds;
}

void StateCollection::setRobots(vector<const supplementary::AgentID*> robots) {
    this->robotIds = robots;
}

vector<State*>& StateCollection::getStates() {
    return states;
}

StateCollection::StateCollection(vector<const supplementary::AgentID*> robots, vector<State*> states) {
    this->robotIds = robots;
    this->states = states;
}

StateCollection::StateCollection(int maxSize) : robotIds(maxSize), states(maxSize) {}

StateCollection::StateCollection(AssignmentCollection* ac) {
    for (int i = 0; i < ac->getSize(); i++) {
        State* initialState = ac->getEp(i)->getState();
        for (auto& robotId : *ac->getRobots(i)) {
            this->setState(robotId, initialState);
        }
    }
}

void StateCollection::setStates(vector<State*> states) {
    this->states = states;
}

int StateCollection::getCount() {
    return this->robotIds.size();
}

State* StateCollection::getState(const supplementary::AgentID* robotId) {
    for (int i = 0; i < this->robotIds.size(); i++) {
        if (*(this->robotIds[i]) == *(robotId)) {
            return this->states[i];
        }
    }
    return nullptr;
}

unordered_set<const supplementary::AgentID*, supplementary::AgentIDHash, supplementary::AgentIDEqualsComparator>
StateCollection::getRobotsInState(State* s) {
    unordered_set<const supplementary::AgentID*, supplementary::AgentIDHash, supplementary::AgentIDEqualsComparator>
            ret;
    for (int i = 0; i < this->robotIds.size(); i++) {
        if (this->states[i] == s) {
            ret.insert(this->robotIds[i]);
        }
    }
    return ret;
}

shared_ptr<vector<const supplementary::AgentID*>> StateCollection::getRobotsInStateSorted(State* s) {
    shared_ptr<vector<const supplementary::AgentID*>> ret = make_shared<vector<const supplementary::AgentID*>>();
    for (int i = 0; i < this->robotIds.size(); i++) {
        if (this->states[i] == s) {
            ret->push_back(this->robotIds[i]);
        }
    }
    sort(ret->begin(), ret->end());
    return ret;
}

unordered_set<const supplementary::AgentID*, supplementary::AgentIDHash, supplementary::AgentIDEqualsComparator>
StateCollection::getRobotsInState(long sid) {
    unordered_set<const supplementary::AgentID*, supplementary::AgentIDHash, supplementary::AgentIDEqualsComparator>
            ret;
    for (int i = 0; i < this->robotIds.size(); i++) {
        if (this->states[i]->getId() == sid) {
            ret.insert(this->robotIds[i]);
        }
    }

    return ret;
}

void StateCollection::removeRobot(const supplementary::AgentID* robotId) {
    for (int i = 0; i < this->states.size(); i++) {
        if (*(this->robotIds[i]) == *(robotId)) {
            this->robotIds.erase(robotIds.begin() + i);
            this->states.erase(states.begin() + i);
            return;
        }
    }
}

void StateCollection::clear() {
    this->robotIds.clear();
    this->states.clear();
}

State* StateCollection::stateOfRobot(const supplementary::AgentID* robotId) {
    for (int i = 0; i < this->robotIds.size(); i++) {
        if (*(this->robotIds[i]) == *robotId) {
            return this->states[i];
        }
    }
    return nullptr;
}

void StateCollection::setState(const supplementary::AgentID* robotId, State* state) {
    for (int i = 0; i < this->robotIds.size(); i++) {
        if (*(this->robotIds[i]) == *robotId) {
            this->states[i] = state;
            return;
        }
    }
    this->robotIds.push_back(robotId);
    this->states.push_back(state);
}

string StateCollection::toString() {
    stringstream ss;
    for (int i = 0; i < robotIds.size(); i++) {
        ss << "R: " << *this->robotIds[i] << " in State: ";
        if (this->states[i] == nullptr) {
            ss << "NULL" << endl;
        } else {
            ss << this->states[i]->getName() << " (" << this->states[i]->getId() << ") " << endl;
        }
    }
    return ss.str();
}

void StateCollection::setInitialState(const supplementary::AgentID* robotId, EntryPoint* ep) {
    setState(robotId, ep->getState());
}

void StateCollection::setStates(vector<const supplementary::AgentID*> robotIds, State* state) {
    for (int i = 0; i < robotIds.size(); i++) {
        setState(robotIds[i], state);
    }
}

/**
 * We are at new assignment, so everything is set to initial states, set them back:
 */
void StateCollection::reconsiderOldAssignment(shared_ptr<Assignment> oldOne, shared_ptr<Assignment> newOne) {
    if (oldOne->getPlan() != newOne->getPlan()) {
        return;
    }
    // shared_ptr<vector<EntryPoint*> >eps = oldOne->getEntryPoints();
    EntryPoint* ep;
    for (short i = 0; i < oldOne->getEntryPointCount(); i++) {
        ep = oldOne->getEpRobotsMapping()->getEp(i);
        for (const supplementary::AgentID*& rid : *(oldOne->getRobotsWorking(ep))) {
            auto iter = find(newOne->getRobotsWorking(ep)->begin(), newOne->getRobotsWorking(ep)->end(), rid);
            if (iter != newOne->getRobotsWorking(ep)->end()) {
                this->setState(rid, oldOne->getRobotStateMapping()->getState(rid));
            }
        }
    }
}

} /* namespace alica */
