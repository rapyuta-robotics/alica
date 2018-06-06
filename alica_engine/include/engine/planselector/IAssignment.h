#pragma once

#include "engine/Types.h"

#include "engine/collections/SuccessCollection.h"
#include "engine/planselector/PartialAssignment.h"
#include "engine/planselector/TaskAssignment.h"

#include <list>
#include <memory>
#include <string>
#include <supplementary/AgentID.h>

namespace alica
{

class EntryPoint;
class SuccessCollection;

class IAssignment;
class AssignmentIterator;
class AssignmentView;
class AssignmentSuccessIterator;
class AssignmentSuccessView;
class UniqueAssignmentSuccessIterator;
class UniqueAssignmentSuccessView;
/**
 *  An IAssignment describes a potentially partial assignment of robots to EntryPoints within a plan.
 */
class IAssignment
{
public:
    IAssignment(const PartialAssignment* pa)
            : _impl(pa)
    {
    }
    int getTotalAgentCount() const { return _impl->getTotalAgentCount(); }

    AssignmentView getRobotsWorking(const EntryPoint* ep) const;
    AssignmentView getRobotsWorking(int64_t epid) const;

    AssignmentView getUnassignedAgents() const;

    AssignmentSuccessView getRobotsWorkingAndFinished(const EntryPoint* ep) const;
    AssignmentSuccessView getRobotsWorkingAndFinished(int64_t epid) const;

    UniqueAssignmentSuccessView getUniqueRobotsWorkingAndFinished(const EntryPoint* ep) const;

private:
    const PartialAssignment* _impl;
};

//-----------------------------View & Iterator classes below
// All iterators are const, as this is a const interface.
class AssignmentIterator
{
public:
    AssignmentIterator(int agentIdx, int epIdx, const PartialAssignment* pas)
            : _pas(pas)
            , _agentIdx(agentIdx)
            , _epIdx(epIdx)
    {
        toNextValid();
    }
    AgentIDConstPtr operator*() const { return _pas->getProblem()->getAgents()[_agentIdx]; }
    AssignmentIterator& operator++()
    {
        ++_agentIdx;
        toNextValid();
        return *this;
    }
    bool operator==(const AssignmentIterator& o) const { return _epIdx == o._epIdx; }

private:
    void toNextValid()
    {
        while (_agentIdx < _pas->getTotalAgentCount() && _pas->getEntryPointIndexOf(_agentIdx) != _epIdx) {
            ++_agentIdx;
        }
    }
    const PartialAssignment* _pas;
    int _agentIdx;
    int _epIdx;
};

class AssignmentView
{
public:
    AssignmentView(int epIdx, const PartialAssignment* pas)
            : _epIdx(epIdx)
            , _pas(pas)
    {
    }

    AssignmentIterator begin() const { return AssignmentIterator(0, _epIdx, _pas); }
    AssignmentIterator end() const { return AssignmentIterator(_pas->getTotalAgentCount(), _epIdx, _pas); }

private:
    const PartialAssignment* _pas;
    int _epIdx;
};

class AssignmentSuccessIterator
{
public:
    AssignmentSuccessIterator(int idx, bool successRange, int epIdx, const PartialAssignment* pas)
            : _pas(pas)
            , _epIdx(epIdx)
            , _agentIdx(idx)
            , _inSuccessRange(successRange)
    {
        toNextValid();
    }
    AgentIDConstPtr operator*() const
    {
        if (_inSuccessRange) {
            return *(_pas->getSuccessData()->getAgentsByIndex(_epIdx))[_agentIdx];
        } else {
            return _pas->getProblem()->getAgents()[_agentIdx];
        }
    }
    AssignmentSuccessIterator& operator++()
    {
        ++_agentIdx;
        toNextValid();
        return *this;
    }
    bool operator==(const AssignmentSuccessIterator& o) const { return _agentIdx == o._agentIdx && _inSuccessRange == o._inSuccessRange; }

private:
    void toNextValid()
    {
        if (!_inSuccessRange) {
            while (_agentIdx < _pas->getTotalAgentCount() && _pas->getEntryPointIndexOf(_agentIdx) != _epIdx) {
                ++_agentIdx;
            }
            if (_agentIdx >= _pas->getTotalAgentCount()) {
                _agentIdx = 0;
                _inSuccessRange = true;
            }
        }
    }
    const PartialAssignment* _pas;
    int _epIdx;
    int _agentIdx;
    bool _inSuccessRange;
};

class UniqueAssignmentSuccessIterator
{
public:
    UniqueAssignmentSuccessIterator(int idx, bool successRange, int epIdx, const PartialAssignment* pas)
            : _pas(pas)
            , _epIdx(epIdx)
            , _agentIdx(idx)
            , _inSuccessRange(successRange)
    {
        toNextValid();
    }
    AgentIDConstPtr operator*() const
    {
        if (_inSuccessRange) {
            return *(_pas->getSuccessData()->getAgentsByIndex(_epIdx))[_agentIdx];
        } else {
            return _pas->getProblem()->getAgents()[_agentIdx];
        }
    }
    UniqueAssignmentSuccessIterator& operator++()
    {
        ++_agentIdx;
        toNextValid();
        return *this;
    }
    bool operator==(const UniqueAssignmentSuccessIterator& o) const { return _agentIdx == o._agentIdx && _inSuccessRange == o._inSuccessRange; }

private:
    void toNextValid();
    const PartialAssignment* _pas;
    int _epIdx;
    int _agentIdx;
    bool _inSuccessRange;
};

class AssignmentSuccessView
{
public:
    AssignmentSuccessView(int epIdx, const PartialAssignment* pas)
            : : _epIdx(epIdx)
                , _pas(pas)
    {
    }
    AssignmentSuccessIterator begin() const { return AssignmentSuccessIterator(0, false, _epidx, _pas); }
    AssignmentSuccessIterator end() const;

private:
    int _epidx;
    const PartialAssignment* _pas;
};

class UniqueAssignmentSuccessView
{
public:
    UniqueAssignmentSuccessView(int epIdx, const PartialAssignment* pas)
            : : _epIdx(epIdx)
                , _pas(pas)
    {
    }
    UniqueAssignmentSuccessIterator begin() const { return AssignmentSuccessIterator(0, false, _epidx, _pas); }
    UniqueAssignmentSuccessIterator end() const;

private:
    int _epidx;
    const PartialAssignment* _pas;
};

UniqueAssignmentSuccessView

} /* namespace alica */
