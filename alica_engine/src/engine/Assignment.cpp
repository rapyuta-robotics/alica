#include "engine/Assignment.h"

#include "engine/collections/SuccessCollection.h"
#include "engine/containers/AllocationAuthorityInfo.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Plan.h"
#include "engine/model/State.h"
#include "engine/model/Task.h"
#include "engine/planselector/PartialAssignment.h"
#include "engine/planselector/TaskAssignmentProblem.h"
#include <supplementary/AgentID.h>

#include <assert.h>

namespace alica
{

bool AgentStatePairs::hasAgent(const AgentIDConstPtr id) const
{
    return std::find_if(_data.begin(), _data.end(), [id](AgentStatePair asp) { return asp.first == id; }) != _data.end();
}

const State* AgentStatePairs::getStateOfAgent(const AgentIDConstPtr id) const
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

void AgentStatePairs::setStateOfAgent(const AgentIDConstPtr id, const State* s)
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

Assignment::Assignment(const Plan* p)
        : _plan(p)
        , _assignmentData(p->getEntryPoints().size())
        , _successData(p->getEntryPoints().size())
        , _lastUtility(0.0)

{
}

Assignment::Assignment(const PartialAssignment& pa)
        : _plan(pa.getPlan())
        , _assignmentData(pa.getPlan()->getEntryPoints().size())
        , _successData(pa.getPlan()->getEntryPoints().size())
        , _lastUtility(pa.getUtility().getMax())
{
    const int numEps = _plan->getEntryPoints().size();

    for (int i = 0; i < numEps; ++i) {
        _assignmentData[i].editRaw().reserve(pa.getAssignedAgentCount(i));
    }
    for (int i = 0; i < pa.getTotalAgentCount(); ++i) {
        const int idx = pa.getEntryPointIndexOf(i);
        if (idx >= 0 && idx < numEps) {
            _assignmentData[idx].emplace_back(pa.getProblem()->getAgents()[i], _plan->getEntryPoints()[i]->getState());
        }
    }
    _successData = pa.getSuccessData()->getRaw();
}

Assignment::Assignment(const Plan* p, const AllocationAuthorityInfo& aai)
        : _plan(p)
        , _assignmentData(p->getEntryPoints().size())
        , _successData(p->getEntryPoints().size())
        , _lastUtility(0.0)
{
    const int numEps = _plan->getEntryPoints().size();
    for (int i = 0; i < numEps; ++i) {
        assert(p->getEntryPoints()[i]->getId() == aai.entryPointRobots[i].entrypoint);
        _assignmentData[i].editRaw().reserve(aai.entryPointRobots[i].robots.size());
        for (AgentIDConstPtr agent : aai.entryPointRobots[i].robots) {
            _assignmentData[i].emplace_back(agent, _plan->getEntryPoints()[i]->getState());
        }
    }
}

Assignment::Assignment(const Assignment& o)
        : _plan(o._plan)
        , _assignmentData(o._assignmentData)
        , _successData(o._successData)
        , _lastUtility(o._lastUtility)
{
}

Assignment& Assignment::operator=(const Assignment& o)
{
    _plan = o._plan;
    _assignmentData = o._assignmentData;
    _successData = o._successData;

    return *this;
}

bool Assignment::isValid() const
{
    if (!_plan) {
        return false;
    }
    const EntryPointGrp& eps = _plan->getEntryPoints();
    const int numEps = eps.size();
    for (int i = 0; i < numEps; ++i) {
        int c = _assignmentData[i].size() + _successData[i].size();
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
    for (int i = 0; i < numEps; ++i) {
        if (eps[i]->isSuccessRequired()) {
            if (!(_successData[i].empty() && static_cast<int>(_successData[i].size()) >= eps[i]->getMinCardinality())) {
                return false;
            }
        }
    }
    return true;
}

bool Assignment::isAnyTaskSuccessful() const
{
    if (!_plan) {
        return false;
    }
    const EntryPointGrp& eps = _plan->getEntryPoints();
    const int numEps = eps.size();
    for (int i = 0; i < numEps; ++i) {
        if (_successData[i].empty() && static_cast<int>(_successData[i].size()) >= eps[i]->getMinCardinality()) {
            return true;
        }
    }
    return false;
}

bool Assignment::hasAgent(AgentIDConstPtr id) const
{
    for (const AgentStatePairs& asps : _assignmentData) {
        if (asps.hasAgent(id)) {
            return true;
        }
    }
    return false;
}

const EntryPoint* Assignment::getEntryPointOfAgent(AgentIDConstPtr id) const
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

const State* Assignment::getStateOfAgent(AgentIDConstPtr id) const
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
        std::transform(asps.begin(), asps.end(), std::back_inserter(o_agents), [](const AgentStatePair asp) -> AgentIDConstPtr { return asp.first; });
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
    std::transform(asp.begin(), asp.end(), std::back_inserter(o_agents), [](AgentStatePair asp) -> AgentIDConstPtr { return asp.first; });
}

void Assignment::getAgentsWorking(int idx, AgentGrp& o_agents) const
{
    const AgentStatePairs& asp = getAgentStates(idx);
    o_agents.reserve(asp.size());
    std::transform(asp.begin(), asp.end(), std::back_inserter(o_agents), [](AgentStatePair asp) -> AgentIDConstPtr { return asp.first; });
}

void Assignment::getAgentsWorkingAndFinished(const EntryPoint* ep, AgentGrp& o_agents) const
{
    const EntryPointGrp& eps = _plan->getEntryPoints();
    const int numEps = eps.size();
    for (int i = 0; i < numEps; ++i) {
        if (ep == eps[i]) {
            o_agents.reserve(_assignmentData[i].size() + _successData[i].size());
            std::transform(_assignmentData[i].begin(), _assignmentData[i].end(), std::back_inserter(o_agents),
                    [](AgentStatePair asp) -> AgentIDConstPtr { return asp.first; });
            std::copy(_successData[i].begin(), _successData[i].end(), std::back_inserter(o_agents));
            return;
        }
    }
}

void Assignment::getAgentsInState(const State* s, AgentGrp& o_agents) const
{
    int i = 0;
    for (const AgentStatePairs& asps : _assignmentData) {
        if (s->getEntryPoint() == _plan->getEntryPoints()[i]) {
            for (AgentStatePair asp : asps) {
                if (asp.second == s) {
                    o_agents.push_back(asp.first);
                }
            }
            break;
        }
        ++i;
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

AssignmentSuccessView Assignment::getAgentsWorkingAndFinished(const EntryPoint* ep) const
{
    return AssignmentSuccessView(this, ep->getIndex());
}

AllAgentsView Assignment::getAllAgents() const
{
    return AllAgentsView(this);
}
AgentsInStateView Assignment::getAgentsInState(const State* s) const
{
    return AgentsInStateView(this, s);
}

void Assignment::clear()
{
    _successData.clear();
    _assignmentData.clear();
}

bool Assignment::updateAgent(AgentIDConstPtr agent, const EntryPoint* e)
{
    return updateAgent(agent, e, nullptr);
}

bool Assignment::updateAgent(AgentIDConstPtr agent, const EntryPoint* e, const State* s)
{
    bool found = false;
    bool inserted = false;
    int i = 0;

    for (AgentStatePairs& asps : _assignmentData) {
        const bool isTargetEp = e == _plan->getEntryPoints()[i];
        if (isTargetEp) {
            for (AgentStatePair asp : asps) {
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
        const bool isTargetEp = ep == _plan->getEntryPoints()[i];
        if (isTargetEp) {
            const State* s = ep->getState();
            for (AgentIDConstPtr id : agents) {
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
            AgentIDConstPtr id = _assignmentData[i].getRaw()[j].first;
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
            AgentIDConstPtr id = _assignmentData[i].getRaw()[j].first;
            if (std::find(limit.begin(), limit.end(), id) == limit.end()) {
                ret = ret || _assignmentData[i].getRaw()[j].second == watchState;
                _assignmentData[i].removeAt(j);
            }
        }
    }
    return ret;
}
void Assignment::removeAgent(AgentIDConstPtr agent)
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
    out << "Assignment:" << std::endl;
    for (int i = 0; i < numEps; ++i) {
        out << "EP: " << eps[i]->getId() << " Task: " << eps[i]->getTask()->getName() << " AgentIDs: ";
        for (AgentStatePair rsp : a._assignmentData[i]) {
            out << rsp.first << "(" << rsp.second->getName() << ") ";
        }
        out << std::endl;
    }
    out << "Success" << std::endl;
    for (int i = 0; i < numEps; ++i) {
        out << "EP: " << eps[i]->getId() << " Task: " << eps[i]->getTask()->getName() << " AgentIDs: ";
        for (AgentIDConstPtr id : a._successData[i]) {
            out << *id << " ";
        }
        out << std::endl;
    }
    return out;
}

/*

void Assignment::getAllRobotsSorted(AgentGrp& o_robots)
{
    getAllRobots(o_robots);
    sort(o_robots.begin(), o_robots.end(), supplementary::AgentIDComparator());
}


bool Assignment::isEqual(Assignment* otherAssignment)
{
    if (otherAssignment == nullptr) {
        return false;
    }
    // check for same length
    if (this->epRobotsMapping->getSize() != otherAssignment->epRobotsMapping->getSize()) {
        return false;
    }
    // check for same entrypoints
    for (int i = 0; i < this->epRobotsMapping->getSize(); ++i) {
        if (this->epRobotsMapping->getEp(i)->getId() != otherAssignment->epRobotsMapping->getEp(i)->getId()) {
            return false;
        }
    }
    // check for same robots in entrypoints
    for (short i = 0; i < this->epRobotsMapping->getSize(); ++i) {
        if (this->epRobotsMapping->getRobots(i)->size() != otherAssignment->epRobotsMapping->getRobots(i)->size()) {
            return false;
        }

        for (auto& robot : *(this->epRobotsMapping->getRobots(i))) {
            auto iter = find_if(otherAssignment->epRobotsMapping->getRobots(i)->begin(), otherAssignment->epRobotsMapping->getRobots(i)->end(),
                                [&robot](const supplementary::AgentID* id) { return *robot == *id; });
            if (iter == otherAssignment->epRobotsMapping->getRobots(i)->end()) {
                return false;
            }
        }
    }
    return true;
}


bool Assignment::isEntryPointNonEmpty(const EntryPoint* ep) const
{
    auto r = this->epRobotsMapping->getRobotsByEp(ep);
    if (r != nullptr && r->size() > 0) {
        return true;
    }
    auto epSuc = this->epSucMapping->getRobots(ep);
    return (epSuc != nullptr && epSuc->size() > 0);
}

bool Assignment::updateRobot(const supplementary::AgentID* robot, const EntryPoint* ep, const State* s)
{
    this->robotStateMapping->setState(robot, s);
    bool ret = false;

    for (int i = 0; i < this->epRobotsMapping->getSize(); i++) {
        if (this->epRobotsMapping->getEp(i) == ep) {
            if (find_if(this->epRobotsMapping->editRobots(i)->begin(), this->epRobotsMapping->editRobots(i)->end(),
                        [&robot](const supplementary::AgentID* id) { return *robot == *id; }) != this->epRobotsMapping->editRobots(i)->end()) {
                return false;
            } else {
                this->epRobotsMapping->editRobots(i)->push_back(robot);
                ret = true;
            }
        } else {
            auto iter = find_if(this->epRobotsMapping->editRobots(i)->begin(), this->epRobotsMapping->editRobots(i)->end(),
                                [&robot](const supplementary::AgentID* id) { return *robot == *id; });
            if (iter != this->epRobotsMapping->editRobots(i)->end()) {
                this->epRobotsMapping->editRobots(i)->erase(iter);
                ret = true;
            }
        }
    }
    return ret;
}

bool Assignment::updateRobot(const supplementary::AgentID* robot, const EntryPoint* ep)
{
    bool ret = false;
    for (int i = 0; i < this->epRobotsMapping->getSize(); i++) {
        if (this->epRobotsMapping->getEp(i) == ep) {
            if (find_if(this->epRobotsMapping->editRobots(i)->begin(), this->epRobotsMapping->editRobots(i)->end(),
                        [&robot](const supplementary::AgentID* id) { return *robot == *id; }) != this->epRobotsMapping->editRobots(i)->end()) {
                return false;
            } else {
                this->epRobotsMapping->editRobots(i)->push_back(robot);
                ret = true;
            }
        } else {
            auto iter = find_if(this->epRobotsMapping->editRobots(i)->begin(), this->epRobotsMapping->editRobots(i)->end(),
                                [&robot](const supplementary::AgentID* id) { return *robot == *id; });
            if (iter != this->epRobotsMapping->editRobots(i)->end()) {
                this->epRobotsMapping->editRobots(i)->erase(iter);
                ret = true;
            }
        }
    }
    if (ret) {
        this->robotStateMapping->setState(robot, ep->getState());
    }
    return ret;
}

*/

} /* namespace alica */
