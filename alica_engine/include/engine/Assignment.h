#pragma once
#include <engine/AgentIDConstPtr.h>
#include <engine/Types.h>
#include <engine/model/EntryPoint.h>
#include <engine/model/Plan.h>
#include <engine/model/State.h>
//#include <algorithm>
#include <assert.h>
#include <sstream>
#include <vector>

namespace alica
{

class SuccessCollection;
class PartialAssignment;
struct AllocationAuthorityInfo;

class AssignmentView;
class AssignmentIterator;

class AllAgentsView;
class AllAgentsIterator;

class AgentsInStateIterator;
class AgentsInStateView;

class AssignmentSuccessIterator;
class AssignmentSuccessView;

class AgentStatePairs
{
public:
    AgentStatePairs() {}
    bool hasAgent(const AgentIDConstPtr id) const;
    const State* getStateOfAgent(const AgentIDConstPtr id) const;
    void setStateOfAgent(const AgentIDConstPtr id, const State* s);

    const std::vector<AgentStatePair>& getRaw() const { return _data; }
    std::vector<AgentStatePair>& editRaw() { return _data; }

    int size() const { return static_cast<int>(_data.size()); }
    bool empty() const { return _data.size() == 0; }

    void clear() { _data.clear(); }
    void emplace_back(AgentIDConstPtr id, const State* s) { _data.emplace_back(id, s); }

    void removeAt(int idx) { _data.erase(_data.begin() + idx); }
    void remove(AgentIDConstPtr agent)
    {
        _data.erase(std::find_if(_data.begin(), _data.end(), [agent](AgentStatePair asp) { return asp.first == agent; }));
    }
    void removeAllIn(const AgentGrp& agents);

    std::vector<AgentStatePair>::iterator begin() { return _data.begin(); }
    std::vector<AgentStatePair>::iterator end() { return _data.end(); }
    std::vector<AgentStatePair>::const_iterator begin() const { return _data.begin(); }
    std::vector<AgentStatePair>::const_iterator end() const { return _data.end(); }

private:
    std::vector<AgentStatePair> _data;
};

/**
 * Contains all allocation information for a single plan. This includes the robot-task mapping, robot-state mapping and
 * success information.
 */
class Assignment
{
public:
    Assignment();
    Assignment(const PartialAssignment& pa);
    Assignment(const Plan* p, const AllocationAuthorityInfo& aai);
    Assignment(const Plan* p);

    Assignment(const Assignment& o);
    Assignment& operator=(const Assignment& o);

    const Plan* getPlan() const { return _plan; }

    bool isValid() const;
    bool isSuccessful() const;
    bool isAnyTaskSuccessful() const;

    bool hasAgent(AgentIDConstPtr id) const;

    int getEntryPointCount() const { return static_cast<int>(_assignmentData.size()); }
    const EntryPoint* getEntryPoint(int idx) const { return _plan->getEntryPoints()[idx]; }
    const EntryPoint* getEntryPointOfAgent(AgentIDConstPtr id) const;
    const State* getStateOfAgent(AgentIDConstPtr id) const;

    void getAllAgents(AgentGrp& o_agents) const;
    const AgentStatePairs& getAgentStates(int idx) const { return _assignmentData[idx]; }
    const AgentStatePairs& getAgentStates(const EntryPoint* ep) const;

    const AgentGrp& getSuccessData(int idx) const { return _successData[idx]; }

    AssignmentView getAgentsWorking(const EntryPoint* ep) const;
    AssignmentView getAgentsWorking(int idx) const;
    AssignmentSuccessView getAgentsWorkingAndFinished(const EntryPoint* ep) const;
    AllAgentsView getAllAgents() const;
    AgentsInStateView getAgentsInState(const State* s) const;

    void getAgentsWorking(const EntryPoint* ep, AgentGrp& o_agents) const;
    void getAgentsWorking(int idx, AgentGrp& o_agents) const;
    void getAgentsWorkingAndFinished(const EntryPoint* ep, AgentGrp& o_agents) const;
    double getLastUtilityValue() const { return _lastUtility; }

    void getAgentsInState(const State* s, AgentGrp& o_agents) const;

    bool updateAgent(AgentIDConstPtr agent, const EntryPoint* e);
    bool updateAgent(AgentIDConstPtr agent, const EntryPoint* e, const State* s);
    void addAgent(const AgentIDConstPtr agent, const EntryPoint* e, const State* s) { _assignmentData[e->getIndex()].emplace_back(agent, s); }
    void setAllToInitialState(const AgentGrp& agents, const EntryPoint* e);
    void setState(AgentIDConstPtr agent, const State* s, const EntryPoint* hint) { _assignmentData[hint->getIndex()].setStateOfAgent(agent, s); }
    bool removeAllIn(const AgentGrp& limit, const State* watchState);
    bool removeAllNotIn(const AgentGrp& limit, const State* watchState);
    void removeAgentFrom(AgentIDConstPtr agent, const EntryPoint* ep) { _assignmentData[ep->getIndex()].remove(agent); }
    void removeAgent(AgentIDConstPtr agent);
    void removeAllFrom(const AgentGrp& agents, const EntryPoint* ep) { _assignmentData[ep->getIndex()].removeAllIn(agents); }
    void clear();
    void moveAllFromTo(const EntryPoint* scope, const State* from, const State* to);
    void adaptTaskChangesFrom(const Assignment& as);
    void fillPartial(PartialAssignment& pa) const;

    AgentGrp& editSuccessData(const EntryPoint* ep) { return _successData[ep->getIndex()]; }

private:
    friend std::ostream& operator<<(std::ostream& out, const Assignment& a);
    const Plan* _plan;
    std::vector<AgentStatePairs> _assignmentData;
    std::vector<AgentGrp> _successData;
    double _lastUtility;
};

std::ostream& operator<<(std::ostream& out, const Assignment& a);

class AssignmentIterator : public std::iterator<std::forward_iterator_tag, AgentIDConstPtr>
{
public:
    AssignmentIterator(int idx, const AgentStatePairs* aps)
            : _agents(aps)
            , _idx(idx)
    {
    }
    AgentIDConstPtr operator*() const { return _agents->getRaw()[_idx].first; }
    AssignmentIterator& operator++()
    {
        ++_idx;
        return *this;
    }
    bool operator==(const AssignmentIterator& o) const { return _idx == o._idx; }
    bool operator!=(const AssignmentIterator& o) const { return !(*this == o); }

private:
    const AgentStatePairs* _agents;
    int _idx;
};

class AssignmentView
{
public:
    AssignmentView()
            : _assignment(nullptr)
            , _epIdx(0)
    {
    }
    AssignmentView(const Assignment* a, int epIdx)
            : _assignment(a)
            , _epIdx(epIdx)
    {
    }
    AssignmentIterator begin() const { return AssignmentIterator(0, _assignment ? &_assignment->getAgentStates(_epIdx) : nullptr); }
    AssignmentIterator end() const
    {
        return _assignment ? AssignmentIterator(_assignment->getAgentStates(_epIdx).size(), &_assignment->getAgentStates(_epIdx))
                           : AssignmentIterator(0, nullptr);
    }
    int size() const { return _assignment ? _assignment->getAgentStates(_epIdx).size() : 0; }

private:
    const Assignment* _assignment;
    const int _epIdx;
};

class AllAgentsIterator : public std::iterator<std::forward_iterator_tag, AgentIDConstPtr>
{
public:
    AllAgentsIterator(int epIdx, int agentIdx, const Assignment* a)
            : _assignment(a)
            , _epIdx(epIdx)
            , _agentIdx(agentIdx)
    {
    }
    AgentIDConstPtr operator*() const { return _assignment->getAgentStates(_epIdx).getRaw()[_agentIdx].first; }
    AllAgentsIterator& operator++()
    {
        ++_agentIdx;
        if (_agentIdx >= _assignment->getAgentStates(_epIdx).size()) {
            _agentIdx = 0;
            ++_epIdx;
        }
        return *this;
    }
    bool operator==(const AllAgentsIterator& o) const { return _agentIdx == o._agentIdx && _epIdx == o._epIdx; }
    bool operator!=(const AllAgentsIterator& o) const { return !(*this == o); }

private:
    const Assignment* _assignment;
    int _epIdx;
    int _agentIdx;
};

class AllAgentsView
{
public:
    AllAgentsView()
            : _assignment(nullptr)
    {
    }
    AllAgentsView(const Assignment* a)
            : _assignment(a)
    {
    }
    AllAgentsIterator begin() const { return AllAgentsIterator(0, 0, _assignment); }
    AllAgentsIterator end() const
    {
        return _assignment ? AllAgentsIterator(_assignment->getEntryPointCount() - 1, _assignment->getAgentStates(_assignment->getEntryPointCount() - 1).size(),
                                     _assignment)
                           : AllAgentsIterator(0, 0, nullptr);
    }
    int size() const
    {
        if (!_assignment) {
            return 0;
        }
        int ret = 0;
        for (int i = 0; i < _assignment->getEntryPointCount(); ++i) {
            ret += _assignment->getAgentStates(i).size();
        }
        return ret;
    }

private:
    const Assignment* _assignment;
};

class AgentsInStateIterator : public std::iterator<std::forward_iterator_tag, AgentIDConstPtr>
{
public:
    AgentsInStateIterator(int idx, const State* s, const AgentStatePairs* aps)
            : _agents(aps)
            , _state(s)
            , _idx(idx)
    {
        toNextValid();
    }
    AgentIDConstPtr operator*() const { return _agents->getRaw()[_idx].first; }
    AgentsInStateIterator& operator++()
    {
        ++_idx;
        toNextValid();
        return *this;
    }
    bool operator==(const AgentsInStateIterator& o) const { return _idx == o._idx; }
    bool operator!=(const AgentsInStateIterator& o) const { return !(*this == o); }

private:
    void toNextValid()
    {
        if (_agents) {
            while (_idx < _agents->size() && _agents->getRaw()[_idx].second != _state) {
                ++_idx;
            }
        }
    }
    const AgentStatePairs* _agents;
    const State* _state;
    int _idx;
};

class AgentsInStateView
{
public:
    AgentsInStateView()
            : _assignment(nullptr)
    {
    }
    AgentsInStateView(const Assignment* a, const State* s)
            : _assignment(a)
            , _state(s)
    {
        assert(s->getEntryPoint()->getPlan() == a->getPlan());
    }
    AgentsInStateIterator begin() const
    {
        return _assignment ? AgentsInStateIterator(0, _state, &_assignment->getAgentStates(_state->getEntryPoint()->getIndex()))
                           : AgentsInStateIterator(0, _state, nullptr);
    }
    AgentsInStateIterator end() const
    {
        if (!_assignment) {
            return AgentsInStateIterator(0, _state, nullptr);
        }
        const AgentStatePairs* asp = &_assignment->getAgentStates(_state->getEntryPoint()->getIndex());
        return AgentsInStateIterator(asp->size(), _state, asp);
    }

private:
    const Assignment* _assignment;
    const State* _state;
};

class AssignmentSuccessIterator : public std::iterator<std::forward_iterator_tag, AgentIDConstPtr>
{
public:
    AssignmentSuccessIterator(int idx, bool inSuccess, const AgentStatePairs* aps, const AgentGrp* successes)
            : _agents(aps)
            , _successData(successes)
            , _idx(idx)
            , _inSuccess(inSuccess)
    {
        if (!inSuccess && aps && _idx >= aps->size()) {
            _inSuccess = true;
            _idx = 0;
        }
    }
    AgentIDConstPtr operator*() const
    {
        if (_inSuccess) {
            return (*_successData)[_idx];
        } else {
            return _agents->getRaw()[_idx].first;
        }
    }
    AssignmentSuccessIterator& operator++()
    {
        ++_idx;
        if (!_inSuccess && _agents && _idx >= _agents->size()) {
            _idx = 0;
            _inSuccess = true;
        }

        return *this;
    }
    bool operator==(const AssignmentSuccessIterator& o) const { return _idx == o._idx && _inSuccess == o._inSuccess; }
    bool operator!=(const AssignmentSuccessIterator& o) const { return !(*this == o); }

private:
    const AgentStatePairs* _agents;
    const AgentGrp* _successData;
    int _idx;
    bool _inSuccess;
};

class AssignmentSuccessView
{
public:
    AssignmentSuccessView()
            : _assignment(nullptr)
            , _epIdx(0)
    {
    }
    AssignmentSuccessView(const Assignment* a, int epIdx)
            : _assignment(a)
            , _epIdx(epIdx)
    {
    }
    AssignmentSuccessIterator begin() const
    {
        if (!_assignment) {
            return AssignmentSuccessIterator(0, false, nullptr, nullptr);
        }
        return AssignmentSuccessIterator(0, false, &_assignment->getAgentStates(_epIdx), &_assignment->getSuccessData(_epIdx));
    }
    AssignmentSuccessIterator end() const
    {
        if (!_assignment) {
            return AssignmentSuccessIterator(0, false, nullptr, nullptr);
        }
        return AssignmentSuccessIterator(
                _assignment->getSuccessData(_epIdx).size(), true, &_assignment->getAgentStates(_epIdx), &_assignment->getSuccessData(_epIdx));
    }
    int size() const { return _assignment ? (_assignment->getAgentStates(_epIdx).size() + _assignment->getSuccessData(_epIdx).size()) : 0; }

private:
    const Assignment* _assignment;
    const int _epIdx;
};
} /* namespace alica */
