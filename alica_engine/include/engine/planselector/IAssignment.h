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
class PartialAssignmentIterator;
class PartialAssignmentView;
class PartialAssignmentSuccessIterator;
class PartialAssignmentSuccessView;
class UniquePartialAssignmentSuccessIterator;
class UniquePartialAssignmentSuccessView;
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

    PartialAssignmentView getRobotsWorking(const EntryPoint* ep) const;
    PartialAssignmentView getRobotsWorking(int64_t epid) const;

    PartialAssignmentView getUnassignedAgents() const;

    PartialAssignmentSuccessView getRobotsWorkingAndFinished(const EntryPoint* ep) const;
    PartialAssignmentSuccessView getRobotsWorkingAndFinished(int64_t epid) const;

    UniquePartialAssignmentSuccessView getUniqueRobotsWorkingAndFinished(const EntryPoint* ep) const;

private:
    const PartialAssignment* _impl;
};

//-----------------------------View & Iterator classes below
// All iterators are const, as this is a const interface.
class PartialAssignmentIterator
{
public:
    PartialAssignmentIterator(int agentIdx, int epIdx, const PartialAssignment* pas)
            : _pas(pas)
            , _agentIdx(agentIdx)
            , _epIdx(epIdx)
    {
        toNextValid();
    }
    AgentIDConstPtr operator*() const { return _pas->getProblem()->getAgents()[_agentIdx]; }
    PartialAssignmentIterator& operator++()
    {
        ++_agentIdx;
        toNextValid();
        return *this;
    }
    bool operator==(const PartialAssignmentIterator& o) const { return _epIdx == o._epIdx; }

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

class PartialAssignmentView
{
public:
    PartialAssignmentView(int epIdx, const PartialAssignment* pas)
            : _epIdx(epIdx)
            , _pas(pas)
    {
    }

    PartialAssignmentIterator begin() const { return PartialAssignmentIterator(0, _epIdx, _pas); }
    PartialAssignmentIterator end() const { return PartialAssignmentIterator(_pas->getTotalAgentCount(), _epIdx, _pas); }

private:
    const PartialAssignment* _pas;
    int _epIdx;
};

class PartialAssignmentSuccessIterator
{
public:
    PartialAssignmentSuccessIterator(int idx, bool successRange, int epIdx, const PartialAssignment* pas)
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
    PartialAssignmentSuccessIterator& operator++()
    {
        ++_agentIdx;
        toNextValid();
        return *this;
    }
    bool operator==(const PartialAssignmentSuccessIterator& o) const { return _agentIdx == o._agentIdx && _inSuccessRange == o._inSuccessRange; }

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

class UniquePartialAssignmentSuccessIterator
{
public:
    UniquePartialAssignmentSuccessIterator(int idx, bool successRange, int epIdx, const PartialAssignment* pas)
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
    UniquePartialAssignmentSuccessIterator& operator++()
    {
        ++_agentIdx;
        toNextValid();
        return *this;
    }
    bool operator==(const UniquePartialAssignmentSuccessIterator& o) const { return _agentIdx == o._agentIdx && _inSuccessRange == o._inSuccessRange; }

private:
    void toNextValid();
    const PartialAssignment* _pas;
    int _epIdx;
    int _agentIdx;
    bool _inSuccessRange;
};

class PartialAssignmentSuccessView
{
public:
    PartialAssignmentSuccessView(int epIdx, const PartialAssignment* pas)
            : : _epIdx(epIdx)
                , _pas(pas)
    {
    }
    PartialAssignmentSuccessIterator begin() const { return PartialAssignmentSuccessIterator(0, false, _epidx, _pas); }
    PartialAssignmentSuccessIterator end() const;

private:
    int _epidx;
    const PartialAssignment* _pas;
};

class UniquePartialAssignmentSuccessView
{
public:
    UniquePartialAssignmentSuccessView(int epIdx, const PartialAssignment* pas)
            : : _epIdx(epIdx)
                , _pas(pas)
    {
    }
    UniquePartialAssignmentSuccessIterator begin() const { return UniquePartialAssignmentSuccessIterator(0, false, _epidx, _pas); }
    UniquePartialAssignmentSuccessIterator end() const;

private:
    int _epidx;
    const PartialAssignment* _pas;
};

} /* namespace alica */
