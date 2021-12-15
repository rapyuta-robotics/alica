#include "engine/Assignment.h"

#include "engine/collections/SuccessCollection.h"
#include "engine/containers/AllocationAuthorityInfo.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Plan.h"
#include "engine/model/State.h"
#include "engine/model/Task.h"
#include "engine/planselector/PartialAssignment.h"
#include "engine/planselector/TaskAssignmentProblem.h"
#include "engine/Types.h"
#include "engine/util/HashFunctions.h"

#include <assert.h>

namespace alica
{

bool AgentStatePairs::hasAgent(const AgentId id) const
{
    return std::find_if(_data.begin(), _data.end(), [id](AgentStatePair asp) { return asp.first == id; }) != _data.end();
}

const State* AgentStatePairs::getStateOfAgent(const AgentId id) const
{
    auto it = std::find_if(_data.begin(), _data.end(), [id](AgentStatePair asp) { return asp.first == id; });
    return it == _data.end() ? nullptr : it->second;
}

void AgentStatePairs::removeAllIn(const AgentGrp& agents)
{
    _data.erase(std::remove_if(_data.begin(), _data.end(),
                        [&agents](const AgentStatePair asp) { return agents.end() != std::find(agents.begin(), agents.end(), asp.first); }),
            _data.end());
}

void AgentStatePairs::setStateOfAgent(const AgentId id, const State* s)
{
    auto it = std::find_if(_data.begin(), _data.end(), [id](AgentStatePair asp) { return asp.first == id; });
    if (it != _data.end()) {
        it->second = s;
    }
}

///------------------------------------------------------------------------------
Assignment::Assignment()
        : _plan(nullptr)
        , _assignmentData()
        , _successData()
        , _lastUtility(0.0)
{
}

Assignment::Assignment(std::size_t parentContextHash, const Plan* p)
        : _plan(p)
        , _assignmentData(p->getEntryPoints().size())
        , _successData(parentContextHash, p)
        , _lastUtility(0.0)

{
}

Assignment::Assignment(const PartialAssignment& pa)
        : _plan(pa.getPlan())
        , _assignmentData(pa.getPlan()->getEntryPoints().size())
        , _successData(*pa.getSuccessData())
        , _lastUtility(pa.getUtility().getMax())
{
    const int numEps = _plan->getEntryPoints().size();

    for (int i = 0; i < numEps; ++i) {
        _assignmentData[i].editRaw().reserve(pa.getAssignedAgentCount(i));
    }
    for (int i = 0; i < pa.getTotalAgentCount(); ++i) {
        const int idx = pa.getEntryPointIndexOf(i);
        if (idx >= 0 && idx < numEps) {
            _assignmentData[idx].emplace_back(pa.getProblem()->getAgents()[i], _plan->getEntryPoints()[idx]->getState());
        }
    }
}

Assignment::Assignment(std::size_t parentContextHash, const Plan* p, const AllocationAuthorityInfo& aai)
        : _plan(p)
        , _assignmentData(p->getEntryPoints().size())
        , _successData(parentContextHash, p)
        , _lastUtility(0.0)
{
    const int numEps = _plan->getEntryPoints().size();
    for (int i = 0; i < numEps; ++i) {
        assert(p->getEntryPoints()[i]->getId() == aai.entryPointRobots[i].entrypoint);
        _assignmentData[i].editRaw().reserve(aai.entryPointRobots[i].robots.size());
        for (AgentId agent : aai.entryPointRobots[i].robots) {
            _assignmentData[i].emplace_back(agent, _plan->getEntryPoints()[i]->getState());
        }
    }
}

bool Assignment::isValid() const
{
    if (!_plan) {
        return false;
    }
    const EntryPointGrp& eps = _plan->getEntryPoints();
    const int numEps = eps.size();
    for (int i = 0; i < numEps; ++i) {
        int c = _assignmentData[i].size() + _successData.getRaw()[i].size();
        if (!eps[i]->getCardinality().contains(c)) {
            return false;
        }
    }
    return true;
}

bool Assignment::isSuccessful() const
{
    if (!_plan) {
        return false;
    }
    const EntryPointGrp& eps = _plan->getEntryPoints();
    const int numEps = eps.size();
    bool ret = false;
    for (int i = 0; i < numEps; ++i) {
        if (eps[i]->isSuccessRequired()) {
            if (_successData.getRaw()[i].empty() || static_cast<int>(_successData.getRaw()[i].size()) < eps[i]->getMinCardinality()) {
                return false;
            }
            ret = true; // Only a plan with successRequired can succeed.
        }
    }
    return ret;
}

bool Assignment::isAnyTaskSuccessful() const
{
    if (!_plan) {
        return false;
    }
    const EntryPointGrp& eps = _plan->getEntryPoints();
    const int numEps = eps.size();
    for (int i = 0; i < numEps; ++i) {
        if (!_successData.getRaw()[i].empty() && static_cast<int>(_successData.getRaw()[i].size()) >= eps[i]->getMinCardinality()) {
            return true;
        }
    }
    return false;
}

bool Assignment::isAgentSuccessful(AgentId id, const EntryPoint* ep) const
{
    if (!_plan) {
        return false;
    }
    const AgentGrp* ag = _successData.getAgents(ep);
    return ag && std::find(ag->begin(), ag->end(), id) != ag->end();
}

bool Assignment::hasAgent(AgentId id) const
{
    for (const AgentStatePairs& asps : _assignmentData) {
        if (asps.hasAgent(id)) {
            return true;
        }
    }
    return false;
}

const EntryPoint* Assignment::getEntryPointOfAgent(AgentId id) const
{
    int i = 0;
    for (const AgentStatePairs& asps : _assignmentData) {
        if (asps.hasAgent(id)) {
            return _plan->getEntryPoints()[i];
        }
        ++i;
    }
    return nullptr;
}

const State* Assignment::getStateOfAgent(AgentId id) const
{
    for (const AgentStatePairs& asps : _assignmentData) {
        const State* s = asps.getStateOfAgent(id);
        if (s) {
            return s;
        }
    }
    return nullptr;
}

void Assignment::getAllAgents(AgentGrp& o_agents) const
{
    for (const AgentStatePairs& asps : _assignmentData) {
        std::transform(asps.begin(), asps.end(), std::back_inserter(o_agents), [](const AgentStatePair asp) -> AgentId { return asp.first; });
    }
}

const AgentStatePairs& Assignment::getAgentStates(const EntryPoint* ep) const
{
    return _assignmentData[ep->getIndex()];
}

void Assignment::getAgentsWorking(const EntryPoint* ep, AgentGrp& o_agents) const
{
    const AgentStatePairs& asp = getAgentStates(ep);
    o_agents.reserve(asp.size());
    std::transform(asp.begin(), asp.end(), std::back_inserter(o_agents), [](AgentStatePair asp) -> AgentId { return asp.first; });
}

void Assignment::getAgentsWorking(int idx, AgentGrp& o_agents) const
{
    const AgentStatePairs& asp = getAgentStates(idx);
    o_agents.reserve(asp.size());
    std::transform(asp.begin(), asp.end(), std::back_inserter(o_agents), [](AgentStatePair asp) -> AgentId { return asp.first; });
}

void Assignment::getAgentsWorkingAndFinished(const EntryPoint* ep, AgentGrp& o_agents) const
{
    const EntryPointGrp& eps = _plan->getEntryPoints();
    const int numEps = eps.size();
    for (int i = 0; i < numEps; ++i) {
        if (ep == eps[i]) {
            o_agents.reserve(_assignmentData[i].size() + _successData.getRaw()[i].size());
            std::transform(_assignmentData[i].begin(), _assignmentData[i].end(), std::back_inserter(o_agents),
                    [](AgentStatePair asp) -> AgentId { return asp.first; });
            std::copy(_successData.getRaw()[i].begin(), _successData.getRaw()[i].end(), std::back_inserter(o_agents));
            return;
        }
    }
}

void Assignment::getAgentsInState(int64_t dynamicEpId, const State* s, AgentGrp& o_agents) const
{
    const EntryPoint* ep = s->getEntryPoint(dynamicEpId);
    assert(ep->isDynamic() == (dynamicEpId > 0));
    for (AgentStatePair asp : _assignmentData[ep->getIndex()]) {
        if (asp.second == s) {
            o_agents.push_back(asp.first);
        }
    }
}

AssignmentView Assignment::getAgentsWorking(const EntryPoint* ep) const
{
    return AssignmentView(this, ep->getIndex());
}

AssignmentView Assignment::getAgentsWorking(int idx) const
{
    return AssignmentView(this, idx);
}

AssignmentView Assignment::getAgentsWorking(int64_t epId) const
{
    const EntryPoint* ep = _plan->getEntryPointByID(epId);
    return ep ? AssignmentView(this, ep->getIndex()) : AssignmentView();
}

AssignmentSuccessView Assignment::getAgentsWorkingAndFinished(const EntryPoint* ep) const
{
    return AssignmentSuccessView(this, ep->getIndex());
}

AllAgentsView Assignment::getAllAgents() const
{
    return AllAgentsView(this);
}
AgentsInStateView Assignment::getAgentsInState(int64_t dynamicEpId, const State* s) const
{
    return AgentsInStateView(this, dynamicEpId, s);
}

AgentsInStateView Assignment::getAgentsInState(int64_t dynamicEpId, int64_t sid) const
{
    const State* s = _plan->getStateByID(sid);
    return s ? AgentsInStateView(this, dynamicEpId, s) : AgentsInStateView();
}

void Assignment::clear()
{
    _successData.clear();
    for (AgentStatePairs& asp : _assignmentData) {
        asp.clear();
    }
}

bool Assignment::updateAgent(AgentId agent, const EntryPoint* e)
{
    return updateAgent(agent, e, nullptr);
}

bool Assignment::updateAgent(AgentId agent, const EntryPoint* e, const State* s)
{
    bool found = false;
    bool inserted = false;
    int i = 0;
    assert(s == nullptr || s->getEntryPoint(e->getDynamicId())->getId() == e->getId());

    for (AgentStatePairs& asps : _assignmentData) {
        const bool isTargetEp = e == _plan->getEntryPoints()[i];
        if (isTargetEp) {
            for (AgentStatePair& asp : asps) {
                if (asp.first == agent) {
                    // assume a null state signals no change
                    if (s == nullptr || asp.second == s) {
                        return false;
                    } else {
                        asp.second = s;
                        return true;
                    }
                }
            }
            asps.emplace_back(agent, s ? s : e->getState());
            inserted = true;
        } else if (!found) {
            for (int j = 0; j < static_cast<int>(asps.size()); ++j) {
                if (asps.getRaw()[j].first == agent) {
                    found = true;
                    asps.removeAt(j);
                    break;
                }
            }
        }
        if (found && inserted) {
            return true;
        }
        ++i;
    }
    return inserted;
}

void Assignment::moveAllFromTo(const EntryPoint* scope, const State* from, const State* to)
{
    assert(from->getEntryPoint(scope->getDynamicId()) == scope);
    assert(to->getEntryPoint(scope->getDynamicId()) == scope);
    for (int i = 0; i < static_cast<int>(_assignmentData.size()); ++i) {
        if (scope == _plan->getEntryPoints()[i]) {
            for (AgentStatePair& asp : _assignmentData[i]) {
                if (asp.second == from) {
                    asp.second = to;
                }
            }
        }
    }
}

void Assignment::setAllToInitialState(const AgentGrp& agents, const EntryPoint* ep)
{
    for (int i = 0; i < static_cast<int>(_assignmentData.size()); ++i) {
        const bool isTargetEp = ep->getIndex() == i;
        if (isTargetEp) {
            const State* s = ep->getState();
            for (AgentId id : agents) {
                auto it = std::find_if(_assignmentData[i].begin(), _assignmentData[i].end(), [id](AgentStatePair asp) { return asp.first == id; });
                if (it == _assignmentData[i].end()) {
                    _assignmentData[i].emplace_back(id, s);
                } else {
                    it->second = s;
                }
            }
        } else {
            _assignmentData[i].removeAllIn(agents);
        }
    }
}

void Assignment::adaptTaskChangesFrom(const Assignment& as)
{
    if (as.getPlan() != getPlan()) {
        *this = as;
        return;
    }
    const int epCount = _assignmentData.size();
    for (int i = 0; i < epCount; ++i) {
        AgentStatePairs n = as._assignmentData[i];
        for (AgentStatePair& asp : n) {
            const State* s = _assignmentData[i].getStateOfAgent(asp.first);
            if (s) {
                asp.second = s;
            }
        }
        _assignmentData[i] = std::move(n);
    }
}

bool Assignment::removeAllIn(const AgentGrp& limit, const State* watchState)
{
    bool ret = false;
    const int epCount = _assignmentData.size();
    for (int i = 0; i < epCount; ++i) {
        for (int j = _assignmentData[i].size() - 1; j >= 0; --j) {
            AgentId id = _assignmentData[i].getRaw()[j].first;
            if (std::find(limit.begin(), limit.end(), id) != limit.end()) {
                ret = ret || _assignmentData[i].getRaw()[j].second == watchState;
                _assignmentData[i].removeAt(j);
            }
        }
    }
    return ret;
}

bool Assignment::removeAllNotIn(const AgentGrp& limit, const State* watchState)
{
    bool ret = false;
    const int epCount = _assignmentData.size();
    for (int i = 0; i < epCount; ++i) {
        for (int j = _assignmentData[i].size() - 1; j >= 0; --j) {
            AgentId id = _assignmentData[i].getRaw()[j].first;
            if (std::find(limit.begin(), limit.end(), id) == limit.end()) {
                ret = ret || _assignmentData[i].getRaw()[j].second == watchState;
                _assignmentData[i].removeAt(j);
            }
        }
    }
    return ret;
}
void Assignment::removeAgent(AgentId agent)
{
    const int epCount = _assignmentData.size();
    for (int i = 0; i < epCount; ++i) {
        auto it = std::find_if(_assignmentData[i].begin(), _assignmentData[i].end(), [agent](AgentStatePair asp) { return agent == asp.first; });
        if (it != _assignmentData[i].end()) {
            _assignmentData[i].editRaw().erase(it);
            return;
        }
    }
    return;
}

void Assignment::fillPartial(PartialAssignment& pa) const
{
    const int epCount = _assignmentData.size();
    const AgentGrp& allAgents = pa.getProblem()->getAgents();
    for (int i = 0; i < epCount; ++i) {
        for (const AgentStatePair& asp : _assignmentData[i]) {
            auto it = std::find(allAgents.begin(), allAgents.end(), asp.first);
            if (it != allAgents.end()) {
                int agentIdx = it - allAgents.begin();
                pa.assignUnassignedAgent(agentIdx, i);
            }
        }
    }
}

std::ostream& operator<<(std::ostream& out, const Assignment& a)
{
    out << std::endl;
    const EntryPointGrp& eps = a._plan->getEntryPoints();
    const int numEps = eps.size();
    assert(numEps == static_cast<int>(a._assignmentData.size()));
    for (int i = 0; i < numEps; ++i) {
        out << "EP: " << eps[i]->getId() << " Task: " << eps[i]->getTask()->getName() << " AgentIDs: ";
        for (AgentStatePair rsp : a._assignmentData[i]) {
            out << rsp.first << "(" << rsp.second->getName() << ") ";
        }
        out << std::endl;
    }
    out << a._successData << std::endl;
    return out;
}

} /* namespace alica */
