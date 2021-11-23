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

const std::string EntryPoint::IDLENAME = "IDLE-EP";

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

EntryPoint::EntryPoint(const EntryPoint& ep, int64_t dynamicId)
        : EntryPoint(ep)
{
    _dynamicId = dynamicId;
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

std::string EntryPoint::toString(std::string indent) const
{
    std::stringstream ss;
    ss << indent << "#EntryPoint: Name: " << getName() << " ID: " << getId() << std::endl;
    ss << indent << "\t Cardinality: " << _cardinality << std::endl;
    ss << indent << "\t Task: ";
    if (_task != nullptr) {
        ss << "Name: " << _task->getName() << " ID: " <<  _task->getId() << std::endl;
    } else {
        ss << "null" << std::endl;
    }
    ss << indent << "\t Initial State: ";
    if (_state != nullptr) {
        ss << "Name: " << _state->getName() << " ID: " << _state->getId() << std::endl;
    } else {
        ss << "null" << std::endl;
    }
    ss << indent << "#EndEntryPoint" << std::endl;
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
