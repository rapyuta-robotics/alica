/*
 * EntryPoint.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/EntryPoint.h"
#include "engine/model/State.h"
#include "engine/model/Task.h"
#include "engine/model/Transition.h"
#include <deque>
namespace alica
{

EntryPoint::EntryPoint()
        : _task(nullptr)
        , _state(nullptr)
        , _successRequired(false)
        , _plan(nullptr)
        , _cardinality(0, 0)
        , _index(-1)
{
}

EntryPoint::EntryPoint(int64_t id, const Plan* p, const Task* t, const State* s)
        : AlicaElement(id)
        , _task(t)
        , _state(s)
        , _successRequired(false)
        , _plan(p)
        , _cardinality(0, 0)
        , _index(-1)
{
}

EntryPoint::~EntryPoint() {}

void EntryPoint::computeReachabilitySet()
{
    std::deque<const State*> queue;
    queue.push_front(_state);
    const State* cs = nullptr;
    while (!queue.empty()) {
        cs = queue.front();
        queue.pop_front();
        if (std::find(_reachableStates.begin(), _reachableStates.end(), cs) == _reachableStates.end()) {
            _reachableStates.push_back(cs);
            for (const Transition* t : cs->getOutTransitions()) {
                queue.push_back(t->getOutState());
            }
        }
    }
}

std::string EntryPoint::toString() const
{
    std::stringstream ss;
    ss << "#EntryPoint: " << getName() << " " << getId() << std::endl;
    ss << "\t Cardinality: " << _cardinality << std::endl;
    ss << "\t Task:" << std::endl;
    if (_task != nullptr) {
        ss << "\t" << _task->getId() << " " << _task->getName();
    } else {
        ss << "null";
    }
    ss << std::endl;
    ss << "\t Initial State:" << std::endl;
    if (_state != nullptr) {
        ss << "\t" << _state->getId() << " " << _state->getName();
    } else {
        ss << "null";
    }
    ss << std::endl;
    ss << "#EndEntryPoint" << std::endl;
    return ss.str();
}

bool EntryPoint::compareTo(const EntryPoint* ep1, const EntryPoint* ep2)
{
    return (ep1->getTask()->getId() > ep2->getTask()->getId());
}

void EntryPoint::setTask(Task* task)
{
    _task = task;
}

void EntryPoint::setPlan(Plan* plan)
{
    _plan = plan;
}

void EntryPoint::setSuccessRequired(bool successRequired)
{
    _successRequired = successRequired;
}

void EntryPoint::setState(State* state)
{
    _state = state;
}
bool EntryPoint::isStateReachable(const State* s) const
{
    return std::find(_reachableStates.begin(), _reachableStates.end(), s) != _reachableStates.end();
}

} // namespace alica
