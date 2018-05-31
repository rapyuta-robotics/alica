#include "engine/Assignment.h"

#include "engine/containers/AllocationAuthorityInfo.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Plan.h"
#include "engine/model/State.h"
#include "engine/model/Task.h"
#include "engine/planselector/PartialAssignment.h"

#include <assert.h>

namespace alica
{

Assignment::Assignment(const Plan* p)
        : _plan(p)
        , _assignmentData(p->getEntryPoints().size())
        , _successData(p->getEntryPoints().size())

{
}

Assignment::Assignment(const PartialAssignment& pa)
        : _plan(pa->getPlan())
        , _assignmentData(pa->getPlan()->getEntryPoints().size())
        , _successData(pa->getPlan()->getEntryPoints().size())
{
    const int numEps = _plan->getEntryPoints().size();

    AssignmentCollection* assCol = pa->getEpRobotsMapping();

    for (int i = 0; i < numEps; ++i) {
        _assignmentData[i].reserve(assCol->getRobots(i)->size());
        for (AgentIdConstPtr agent : *assCol->getRobots()) {
            _assignmentData[i].emplace_back(agent, _plan->getEntryPoints()[i]->getState());
        }
    }
}

Assignment::Assignment(const Plan* p, const AllocationAuthorityInfo& aai)
        : _plan(p)
        , _assignmentData(p->getEntryPoints().size())
        , _successData(p->getEntryPoints().size())
{
    const int numEps = _plan->getEntryPoints().size();
    for (int i = 0; i < numEps; ++i) {
        assert(p->getEntryPoints()[i]->getId() == aai.entryPointRobots[i].entrypoint;
         _assignmentData[i].reserve(aai.entryPointRobots[i].robots.size());
        for (AgentIdConstPtr robot : aai.entryPointRobots[i].robots) {
            _assignmentData[i].emplace_back(agent, _plan->getEntryPoints()[i]->getState());
        }
    }
}

Assignment(const Assignment& o)
        : _plan(o._plan)
        , _assignmentData(o._assignmentData)
        , _successData(o._successData)
{
}

Assignment& operator=(const Assignment& o)
{
    _plan = o._plan;
    _assignmentData = o._assignmentData;
    _successData = o._successData;

    return *this;
}

Assignment::~Assignment() {}

bool Assignment::isValid() const
{
    const EntryPointGrp& eps = _plan->getEntryPoints();
    const int numEps = eps.size();
    for (int i = 0; i < numEps; ++i) {
        int c = _assignmentData[i].size() + _successData[i].size();
        if (!eps[i].contains(c)) {
            return false;
        }
    }
    return true;
}

bool Assignment::isSuccessfull() const
{
    const EntryPointGrp& eps = _plan->getEntryPoints();
    const int numEps = eps.size();
    for (int i = 0; i < numEps; ++i) {
        if (eps[i]->isSuccessRequired()) {
            if (!(_successData[i].empty() && static_cast<int>(_successData[i]->size()) >= eps[i]->getMinCardinality())) {
                return false;
            }
        }
    }
    return true;
}

bool Assignment::hasAgent(AgentIDConstPtr id) const
{
    for (const std::vector<AgentStatePair>& asps : _assignmentData) {
        if (std::find_if(asps.begin(), asps.end(), [id](const RobotStatePair asp) { return *asp.first == *id; }) != asps.end()) {
            return true;
        }
    }
    return false;
}

void Assignment::clear()
{
    _successData.clear();
    _assignmentData.clear();
}

void Assignment::getAgentsInState(const State* s, AgentGrp& o_agents)
{
    int i = 0;
    for (const std::vector<AgentStatePair>& asps : _assignmentData) {
        if (s->getEntryPoint() == _plan->getEntryPoint()[i]) {
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
bool updateAgent(AgentIDConstPtr agent, const EntryPoint* e)
{
    bool found = false;
    bool ret = false;
    int i = 0;

    for (const std::vector<AgentStatePair>& asps : _assignmentData) {
        bool isTargetEp = e == _plan->getEntryPoint()[i];
        if (!found) {
            for (int j = 0; j < static_cast<int>(asps.size()); ++j) {
                if (asps[j].first == agent) {
                    if (isTargetEp) {
                        return false;
                    } else {
                        found = true;
                        asps.erase(asps.begin() + j);
                        break;
                    }
                }
            }
        }
        if (isTargetEp) {
            asps.emplace_back(agent, e->getState());
            if (found) {
                return true;
            }
            ret = true;
        }
        ++i;
    }
    return ret;
}

std::ostream& operator<<(std::ostream& out, const Assignment& a)
{
    out << std::endl;
    const EntryPointGrp& eps = a._plan->getEntryPoints();
    const int numEps = eps.size();
    out << "Assignment:" << std::endl;
    for (int i = 0; i < numEps; ++i) {
        out << "EP: " << eps[i]->getId() << " Task: " << eps[i]->getTask()->getName() << " AgentIDs: ";
        for (RobotStatePair rsp : _assignmentData[i])) {
            out << *rsp.first << "("<< *rsp.second->getName()<<") ";
        }
        out << std::endl;
    }
    out << "Success" << std::endl;
    for (int i = 0; i < numEps; ++i) {
        out << "EP: " << eps[i]->getId() << " Task: " << eps[i]->getTask()->getName() << " AgentIDs: ";
        for (AgentIdConstPtr a : _successData[i])) {
            out << *a << " ";
        }
        out << std::endl;
    }

    return out;
}

/*
StateCollection* Assignment::getRobotStateMapping()
{
    return robotStateMapping;
}

void Assignment::getAllRobots(AgentGrp& o_robots)
{
    for (int i = 0; i < this->epRobotsMapping->getSize(); ++i) {
        for (int j = 0; j < static_cast<int>(this->epRobotsMapping->getRobots(i)->size()); ++j) {
            o_robots.push_back(this->epRobotsMapping->getRobots(i)->at(j));
        }
    }
}

void Assignment::getAllRobotsSorted(AgentGrp& o_robots)
{
    getAllRobots(o_robots);
    sort(o_robots.begin(), o_robots.end(), supplementary::AgentIDComparator());
}

const AgentGrp* Assignment::getRobotsWorking(int64_t epid) const
{
    return this->epRobotsMapping->getRobotsByEpId(epid);
}

void Assignment::getRobotsWorkingSorted(const EntryPoint* ep, AgentGrp& o_robots)
{
    o_robots = *getRobotsWorking(ep);
    sort(o_robots.begin(), o_robots.end(), supplementary::AgentIDComparator());
}

const AgentGrp* Assignment::getRobotsWorking(const EntryPoint* ep) const
{
    return this->epRobotsMapping->getRobotsByEp(ep);
}

int Assignment::totalRobotCount() const
{
    int c = 0;
    for (int i = 0; i < this->epRobotsMapping->getSize(); ++i) {
        c += this->epRobotsMapping->getRobots(i)->size();
    }
    return this->getNumUnAssignedRobotIds() + c;
}

short Assignment::getEntryPointCount() const
{
    return this->epRobotsMapping->getSize();
}

shared_ptr<list<const supplementary::AgentID*>> Assignment::getRobotsWorkingAndFinished(const EntryPoint* ep)
{
    auto ret = std::make_shared<list<const supplementary::AgentID*>>(list<const supplementary::AgentID*>());
    auto robots = this->epRobotsMapping->getRobotsByEp(ep);
    if (robots != nullptr) {
        for (int i = 0; i < static_cast<int>(robots->size()); ++i) {
            ret->push_back(robots->at(i));
        }
    }

    auto robotsSucc = this->epSucMapping->getRobots(ep);
    if (robotsSucc != nullptr) {
        for (auto& robot : *robotsSucc) {
            ret->push_back(robot);
        }
    }

    return ret;
}


shared_ptr<list<const supplementary::AgentID*>> Assignment::getUniqueRobotsWorkingAndFinished(const EntryPoint* ep)
{
    auto ret = std::make_shared<list<const supplementary::AgentID*>>(list<const supplementary::AgentID*>());
    // if (this->plan->getEntryPoints().find(ep->getId()) != this->plan->getEntryPoints().end())
    {
        auto robots = this->epRobotsMapping->getRobotsByEp(ep);
        for (int i = 0; i < static_cast<int>(robots->size()); ++i) {
            ret->push_back(robots->at(i));
        }
        for (auto r : (*this->epSucMapping->getRobots(ep))) {
            if (std::find_if(ret->begin(), ret->end(), [&r](const supplementary::AgentID* id) { return *r == *id; }) == ret->end()) {
                ret->push_back(r);
            }
        }
    }
    return ret;
}

std::shared_ptr<list<const supplementary::AgentID*>> Assignment::getRobotsWorkingAndFinished(int64_t epid)
{
    auto ret = std::make_shared<std::list<const supplementary::AgentID*>>();
    auto robots = this->epRobotsMapping->getRobotsByEpId(epid);
    if (robots != nullptr) {
        for (int i = 0; i < static_cast<int>(robots->size()); i++) {
            ret->push_back(robots->at(i));
        }
    }

    auto robotsSucc = this->epSucMapping->getRobotsById(epid);
    if (robotsSucc != nullptr) {
        for (auto& robot : *robotsSucc) {
            ret->push_back(robot);
        }
    }

    return ret;
}

std::shared_ptr<SuccessCollection> Assignment::getEpSuccessMapping() const
{
    return this->epSucMapping;
}

void Assignment::setAllToInitialState(const AgentGrp& robots, const EntryPoint* defep)
{
    auto rlist = this->epRobotsMapping->editRobotsByEp(defep);
    for (const supplementary::AgentID* r : robots) {
        rlist->push_back(r);
        robotStateMapping->setState(r, defep->getState());
    }
}

bool Assignment::removeRobot(const supplementary::AgentID* robotId)
{
    this->robotStateMapping->removeRobot(robotId);
    AgentGrp* curRobots;
    for (int i = 0; i < this->epRobotsMapping->getSize(); ++i) {
        curRobots = this->epRobotsMapping->editRobots(i);
        auto iter = find_if(curRobots->begin(), curRobots->end(), [&robotId](const supplementary::AgentID* id) { return *robotId == *id; });
        if (iter != curRobots->end()) {
            curRobots->erase(iter);
            return true;
        }
    }
    return false;
}

void Assignment::addRobot(const supplementary::AgentID* id, const EntryPoint* e, const State* s)
{
    if (e == nullptr) {
        return;
    }
    this->robotStateMapping->setState(id, s);
    this->epRobotsMapping->addRobot(id, e);
    return;
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

bool Assignment::removeRobot(const supplementary::AgentID* robot, const EntryPoint* ep)
{
    assert(ep != nullptr);
    if (ep == nullptr) {
        return false;
    }
    this->robotStateMapping->removeRobot(robot);
    return epRobotsMapping->removeRobot(robot, ep);
}

std::string Assignment::assignmentCollectionToString() const
{
    std::stringstream ss;
    ss << "ASS" << std::endl;
    ss << this->epRobotsMapping->toString();
    return ss.str();
}

void Assignment::addRobot(const supplementary::AgentID* id, const EntryPoint* e)
{
    assert(e != nullptr);
    if (e == nullptr) {
        return;
    }
    this->epRobotsMapping->addRobot(id, e);
    return;
}

void Assignment::moveRobots(const State* from, const State* to)
{
    assert(to != nullptr);
    robotStateMapping->moveAllFromTo(from, to);
}


const EntryPoint* Assignment::getEntryPointOfRobot(const supplementary::AgentID* robot)
{
    for (int i = 0; i < this->epRobotsMapping->getSize(); i++) {
        auto iter = find_if(this->epRobotsMapping->getRobots(i)->begin(), this->epRobotsMapping->getRobots(i)->end(),
                            [&robot](const supplementary::AgentID* id) { return *robot == *id; });
        if (iter != this->epRobotsMapping->getRobots(i)->end()) {
            return this->epRobotsMapping->getEp(i);
        }
    }
    return nullptr;
}
*/

} /* namespace alica */
