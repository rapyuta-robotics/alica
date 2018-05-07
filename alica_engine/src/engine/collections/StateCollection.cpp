#include "engine/collections/StateCollection.h"
#include "engine/model/State.h"
#include "engine/model/EntryPoint.h"
#include "engine/collections/AssignmentCollection.h"
#include "engine/Assignment.h"

namespace alica {

StateCollection::StateCollection() {}

StateCollection::~StateCollection() {}

StateCollection::StateCollection(const AgentGrp& robots, const StateGrp& states)
        : _robotIds(robots)
        , _states(states) {}

StateCollection::StateCollection(int maxSize)
        : _robotIds(maxSize)
        , _states(maxSize) {}

StateCollection::StateCollection(const AssignmentCollection* ac) {
    for (int i = 0; i < ac->getSize(); ++i) {
        const State* initialState = ac->getEp(i)->getState();
        for (const supplementary::AgentID* robotId : *ac->getRobots(i)) {
            setState(robotId, initialState);
        }
    }
}

const State* StateCollection::getStateOfRobot(const supplementary::AgentID* robotId) const {
    for (int i = 0; i < _robotIds.size(); i++) {
        if (*(_robotIds[i]) == *(robotId)) {
            return _states[i];
        }
    }
    return nullptr;
}

int StateCollection::getRobotsInState(const State* s, AgentGrp& o_robots) const {
    int c = 0;
    for (int i = 0; i < _robotIds.size(); ++i) {
        if (_states[i] == s) {
            o_robots.push_back(_robotIds[i]);
            ++c;
        }
    }
    return c;
}

int StateCollection::getRobotsInState(int64_t sid, AgentGrp& o_robots) const {
    int c = 0;
    for (int i = 0; i < _robotIds.size(); ++i) {
        if (_states[i]->getId() == sid) {
            o_robots.push_back(_robotIds[i]);
            ++c;
        }
    }
    return c;
}

void StateCollection::getRobotsInStateSorted(const State* s, AgentGrp& o_robots) const {
    getRobotsInState(s, o_robots);
    sort(o_robots.begin(), o_robots.end(), supplementary::AgentIDComparator());
}

void StateCollection::removeRobot(const supplementary::AgentID* robotId) {
    for (int i = 0; i < _states.size(); i++) {
        if (*(_robotIds[i]) == *(robotId)) {
            _robotIds.erase(_robotIds.begin() + i);
            _states.erase(_states.begin() + i);
            return;
        }
    }
}

void StateCollection::clear() {
    _robotIds.clear();
    _states.clear();
}

void StateCollection::setState(const supplementary::AgentID* robotId, const State* state) {
    for (int i = 0; i < _robotIds.size(); ++i) {
        if (*(_robotIds[i]) == *robotId) {
            _states[i] = state;
            return;
        }
    }
    _robotIds.push_back(robotId);
    _states.push_back(state);
}

std::string StateCollection::toString() const {
    std::stringstream ss;
    for (int i = 0; i < _robotIds.size(); i++) {
        ss << "R: " << *_robotIds[i] << " in State: ";
        if (_states[i] == nullptr) {
            ss << "NULL" << std::endl;
        } else {
            ss << _states[i]->getName() << " (" << _states[i]->getId() << ") " << std::endl;
        }
    }
    return ss.str();
}

void StateCollection::setInitialState(const supplementary::AgentID* robotId, const EntryPoint* ep) {
    setState(robotId, ep->getState());
}

void StateCollection::setStates(const AgentGrp& robotIds, const State* state) {
    for (const supplementary::AgentID* r : robotIds) {
        setState(r, state);
    }
}

void StateCollection::moveAllFromTo(const State* from, const State* to) {
    for (int i = 0; i < _states.size(); ++i) {
        if (_states[i] == from) {
            _states[i] = to;
        }
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
    const EntryPoint* ep;
    for (short i = 0; i < oldOne->getEntryPointCount(); i++) {
        ep = oldOne->getEpRobotsMapping()->getEp(i);
        for (const supplementary::AgentID* rid : *(oldOne->getRobotsWorking(ep))) {
            auto iter = find(newOne->getRobotsWorking(ep)->begin(), newOne->getRobotsWorking(ep)->end(), rid);
            if (iter != newOne->getRobotsWorking(ep)->end()) {
                setState(rid, oldOne->getRobotStateMapping()->getStateOfRobot(rid));
            }
        }
    }
}

} /* namespace alica */
