#pragma once

#include "engine/Types.h"

#include "engine/collections/SuccessCollection.h"
#include "engine/planselector/PartialAssignment.h"
#include "engine/planselector/TaskAssignmentProblem.h"

#include <supplementary/AgentID.h>

#include <iterator>

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
 *  An IAssignment describes a potentially partial assignment of agents to EntryPoints within a plan.
 */
class IAssignment
{
public:
    explicit IAssignment(const PartialAssignment* pa)
            : _impl(pa)
    {
    }
    int getTotalAgentCount() const { return _impl->getTotalAgentCount(); }
    int getAssignedAgentCount() const { return _impl->getAssignedAgentCount(); }
    int getUnAssignedAgentCount() const { return getTotalAgentCount() - getAssignedAgentCount(); }
    int getEntryPointCount() const { return _impl->getEntryPointCount(); }
    const EntryPoint* getEntryPoint(int idx) const { return _impl->getPlan()->getEntryPoints()[idx]; }

    PartialAssignmentView getAgentsWorking(const EntryPoint* ep) const;
    PartialAssignmentView getAgentsWorking(int64_t epid) const;

    PartialAssignmentView getUnassignedAgents() const;

    PartialAssignmentSuccessView getAgentsWorkingAndFinished(const EntryPoint* ep) const;
    PartialAssignmentSuccessView getAgentsWorkingAndFinished(int64_t epid) const;

    UniquePartialAssignmentSuccessView getUniqueAgentsWorkingAndFinished(const EntryPoint* ep) const;

private:
    const PartialAssignment* _impl;
};

//-----------------------------View & Iterator classes below
// All iterators are const, as this is a const interface.
class PartialAssignmentIterator : public std::iterator<std::forward_iterator_tag, AgentIDConstPtr>
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
    bool operator!=(const PartialAssignmentIterator& o) const { return !(*this == o); }

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
    int size() const { return std::distance(begin(), end()); }

private:
    const PartialAssignment* _pas;
    int _epIdx;
};

class PartialAssignmentSuccessIterator : public std::iterator<std::forward_iterator_tag, AgentIDConstPtr>
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
            return (*_pas->getSuccessData()->getAgentsByIndex(_epIdx))[_agentIdx];
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
    bool operator!=(const PartialAssignmentSuccessIterator& o) const { return !(*this == o); }

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

class UniquePartialAssignmentSuccessIterator : public std::iterator<std::forward_iterator_tag, AgentIDConstPtr>
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
            return (*_pas->getSuccessData()->getAgentsByIndex(_epIdx))[_agentIdx];
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
    bool operator!=(const UniquePartialAssignmentSuccessIterator& o) const { return !(*this == o); }

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
            : _epIdx(epIdx)
            , _pas(pas)
    {
    }
    PartialAssignmentSuccessIterator begin() const { return PartialAssignmentSuccessIterator(0, false, _epIdx, _pas); }
    PartialAssignmentSuccessIterator end() const;
    bool empty() const { return begin() == end(); }
    int size() const { return PartialAssignmentView(_epIdx, _pas).size() + (_pas ? _pas->getSuccessData()->getAgentsByIndex(_epIdx)->size() : 0); }

private:
    int _epIdx;
    const PartialAssignment* _pas;
};

class UniquePartialAssignmentSuccessView
{
public:
    UniquePartialAssignmentSuccessView(int epIdx, const PartialAssignment* pas)
            : _epIdx(epIdx)
            , _pas(pas)
    {
    }
    UniquePartialAssignmentSuccessIterator begin() const { return UniquePartialAssignmentSuccessIterator(0, false, _epIdx, _pas); }
    UniquePartialAssignmentSuccessIterator end() const;

private:
    int _epIdx;
    const PartialAssignment* _pas;
};

} /* namespace alica */
