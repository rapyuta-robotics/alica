#pragma once

#include <engine/Types.h>
#include <engine/collections/SuccessCollection.h>
#include <engine/model/EntryPoint.h>
#include <engine/model/Plan.h>
#include <engine/model/State.h>

#include <essentials/IdentifierConstPtr.h>
#include <assert.h>
#include <numeric>
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
    bool hasAgent(const essentials::IdentifierConstPtr id) const;
    const State* getStateOfAgent(const essentials::IdentifierConstPtr id) const;
    void setStateOfAgent(const essentials::IdentifierConstPtr id, const State* s);

    const std::vector<AgentStatePair>& getRaw() const { return _data; }
    std::vector<AgentStatePair>& editRaw() { return _data; }

    int size() const { return static_cast<int>(_data.size()); }
    bool empty() const { return _data.size() == 0; }

    void clear() { _data.clear(); }
    void emplace_back(essentials::IdentifierConstPtr id, const State* s) { _data.emplace_back(id, s); }

    void removeAt(int idx) { _data.erase(_data.begin() + idx); }
    void remove(essentials::IdentifierConstPtr agent)
    {
        _data.erase(std::find_if(_data.begin(), _data.end(), [agent](AgentStatePair asp) { return asp.first == agent; }));
    }
    void removeAllIn(const AgentGrp& agents);
    template <typename ForwardIterator>
    void removeAllIn(ForwardIterator begin, ForwardIterator end);

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

    Assignment(const Assignment& o) = default;
    Assignment& operator=(const Assignment& o) = default;
    Assignment(Assignment&& o) = default;
    Assignment& operator=(Assignment&& o) = default;

    const Plan* getPlan() const { return _plan; }

    bool isValid() const;
    bool isSuccessful() const;
    bool isAnyTaskSuccessful() const;
    bool isAgentSuccessful(essentials::IdentifierConstPtr id, const EntryPoint* ep) const;

    bool hasAgent(essentials::IdentifierConstPtr id) const;
    int size() const
    {
        return std::accumulate(_assignmentData.begin(), _assignmentData.end(), 0, [](int val, const AgentStatePairs& asps) { return val + asps.size(); });
    }
    int getEntryPointCount() const { return static_cast<int>(_assignmentData.size()); }
    const EntryPoint* getEntryPoint(int idx) const { return _plan->getEntryPoints()[idx]; }
    const EntryPoint* getEntryPointOfAgent(essentials::IdentifierConstPtr id) const;
    const State* getStateOfAgent(essentials::IdentifierConstPtr id) const;

    void getAllAgents(AgentGrp& o_agents) const;
    const AgentStatePairs& getAgentStates(int idx) const { return _assignmentData[idx]; }
    const AgentStatePairs& getAgentStates(const EntryPoint* ep) const;

    const AgentGrp& getSuccessData(int idx) const { return *_successData.getAgentsByIndex(idx); }
    AgentGrp& editSuccessData(const EntryPoint* ep) { return _successData.editAgentsByIndex(ep->getIndex()); }
    SuccessCollection& editSuccessData() { return _successData; }

    AssignmentView getAgentsWorking(const EntryPoint* ep) const;
    AssignmentView getAgentsWorking(int idx) const;
    AssignmentView getAgentsWorking(int64_t epId) const;

    AssignmentSuccessView getAgentsWorkingAndFinished(const EntryPoint* ep) const;
    AllAgentsView getAllAgents() const;
    AgentsInStateView getAgentsInState(const State* s) const;
    AgentsInStateView getAgentsInState(int64_t sid) const;

    void getAgentsWorking(const EntryPoint* ep, AgentGrp& o_agents) const;
    void getAgentsWorking(int idx, AgentGrp& o_agents) const;
    void getAgentsWorkingAndFinished(const EntryPoint* ep, AgentGrp& o_agents) const;
    double getLastUtilityValue() const { return _lastUtility; }

    void getAgentsInState(const State* s, AgentGrp& o_agents) const;

    bool updateAgent(essentials::IdentifierConstPtr agent, const EntryPoint* e);
    bool updateAgent(essentials::IdentifierConstPtr agent, const EntryPoint* e, const State* s);
    void addAgent(essentials::IdentifierConstPtr agent, const EntryPoint* e, const State* s) { _assignmentData[e->getIndex()].emplace_back(agent, s); }
    void setAllToInitialState(const AgentGrp& agents, const EntryPoint* e);
    template <typename ForwardIterator>
    void setAllToInitialState(ForwardIterator begin, ForwardIterator end, const EntryPoint* e);

    void setState(essentials::IdentifierConstPtr agent, const State* s, const EntryPoint* hint) { _assignmentData[hint->getIndex()].setStateOfAgent(agent, s); }
    bool removeAllIn(const AgentGrp& limit, const State* watchState);
    bool removeAllNotIn(const AgentGrp& limit, const State* watchState);
    void removeAgentFrom(essentials::IdentifierConstPtr agent, const EntryPoint* ep) { _assignmentData[ep->getIndex()].remove(agent); }
    void removeAgent(essentials::IdentifierConstPtr agent);
    void removeAllFrom(const AgentGrp& agents, const EntryPoint* ep) { _assignmentData[ep->getIndex()].removeAllIn(agents); }
    void clear();
    void moveAllFromTo(const EntryPoint* scope, const State* from, const State* to);
    void adaptTaskChangesFrom(const Assignment& as);
    void fillPartial(PartialAssignment& pa) const;

private:
    friend std::ostream& operator<<(std::ostream& out, const Assignment& a);
    const Plan* _plan;
    std::vector<AgentStatePairs> _assignmentData;
    SuccessCollection _successData;
    double _lastUtility;
};

std::ostream& operator<<(std::ostream& out, const Assignment& a);

template <typename ForwardIterator>
void Assignment::setAllToInitialState(ForwardIterator begin, ForwardIterator end, const EntryPoint* ep)
{
    for (int i = 0; i < static_cast<int>(_assignmentData.size()); ++i) {
        const bool isTargetEp = ep->getIndex() == i;
        if (isTargetEp) {
            const State* s = ep->getState();
            for (ForwardIterator agent_it = begin; agent_it != end; ++agent_it) {
                essentials::IdentifierConstPtr id = *agent_it;
                auto it = std::find_if(_assignmentData[i].begin(), _assignmentData[i].end(), [id](AgentStatePair asp) { return asp.first == id; });
                if (it == _assignmentData[i].end()) {
                    _assignmentData[i].emplace_back(id, s);
                } else {
                    it->second = s;
                }
            }
        } else {
            _assignmentData[i].removeAllIn(begin, end);
        }
    }
}

template <typename ForwardIterator>
void AgentStatePairs::removeAllIn(ForwardIterator begin, ForwardIterator end)
{
    _data.erase(std::remove_if(_data.begin(), _data.end(), [begin, end](const AgentStatePair asp) { return end != std::find(begin, end, asp.first); }),
            _data.end());
}

class AssignmentIterator : public std::iterator<std::forward_iterator_tag, essentials::IdentifierConstPtr>
{
public:
    AssignmentIterator(int idx, const AgentStatePairs* aps)
            : _agents(aps)
            , _idx(idx)
    {
    }
    essentials::IdentifierConstPtr operator*() const { return _agents->getRaw()[_idx].first; }
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
    bool empty() const { return size() == 0; }

private:
    const Assignment* _assignment;
    const int _epIdx;
};

class AllAgentsIterator : public std::iterator<std::forward_iterator_tag, essentials::IdentifierConstPtr>
{
public:
    AllAgentsIterator(int epIdx, int agentIdx, const Assignment* a)
            : _assignment(a)
            , _epIdx(epIdx)
            , _agentIdx(agentIdx)
    {
        toNextValid();
    }

    essentials::IdentifierConstPtr operator*() const { return _assignment->getAgentStates(_epIdx).getRaw()[_agentIdx].first; }
    AllAgentsIterator& operator++()
    {
        ++_agentIdx;
        toNextValid();
        return *this;
    }

    bool operator==(const AllAgentsIterator& o) const { return _agentIdx == o._agentIdx && _epIdx == o._epIdx; }
    bool operator!=(const AllAgentsIterator& o) const { return !(*this == o); }

private:
    void toNextValid()
    {
        while (_epIdx < _assignment->getEntryPointCount() && _agentIdx >= _assignment->getAgentStates(_epIdx).size()) {
            _agentIdx = 0;
            ++_epIdx;
        }
    }
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
        return _assignment ? AllAgentsIterator(_assignment->getEntryPointCount(), 0, _assignment) : AllAgentsIterator(0, 0, nullptr);
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
    bool empty() const { return size() == 0; }

private:
    const Assignment* _assignment;
};

class AgentsInStateIterator : public std::iterator<std::forward_iterator_tag, essentials::IdentifierConstPtr>
{
public:
    AgentsInStateIterator(int idx, const State* s, const AgentStatePairs* aps)
            : _agents(aps)
            , _state(s)
            , _idx(idx)
    {
        toNextValid();
    }
    essentials::IdentifierConstPtr operator*() const { return _agents->getRaw()[_idx].first; }
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
            , _state(nullptr)
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

    bool empty() const { return begin() == end(); }

private:
    const Assignment* _assignment;
    const State* _state;
};

class AssignmentSuccessIterator : public std::iterator<std::forward_iterator_tag, essentials::IdentifierConstPtr>
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
    essentials::IdentifierConstPtr operator*() const
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
    bool empty() const { return size() == 0; }

private:
    const Assignment* _assignment;
    const int _epIdx;
};
} /* namespace alica */
