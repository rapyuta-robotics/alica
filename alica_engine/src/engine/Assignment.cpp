#include "engine/Assignment.h"

#include "engine/collections/AssignmentCollection.h"
#include "engine/model/Plan.h"
#include "engine/collections/StateCollection.h"
#include "engine/collections/SuccessCollection.h"
#include "engine/model/EntryPoint.h"
#include "engine/planselector/PartialAssignment.h"
#include "engine/containers/AllocationAuthorityInfo.h"
#include "engine/containers/EntryPointRobots.h"
#include "engine/model/Task.h"
#include "engine/model/State.h"

#include <assert.h>

namespace alica {

Assignment::~Assignment() {
    delete this->epRobotsMapping;
    delete this->robotStateMapping;
}
Assignment::Assignment(const Plan* p) {
    this->plan = p;
    this->max = 0.0;
    this->min = 0.0;

    this->epRobotsMapping = new AssignmentCollection(this->plan->getEntryPoints().size());

    // sort the entrypoints of the given plan
    list<const EntryPoint*> sortedEpList;
    for (const EntryPoint* e : plan->getEntryPoints()) {
        sortedEpList.push_back(e);
    }
    sortedEpList.sort(EntryPoint::compareTo);  // TODO: Entrypoints are supposed to be pre-sorted. Confirm and remove

    // add the sorted entrypoints into the assignmentcollection
    short i = 0;
    for (const EntryPoint* ep : sortedEpList) {
        this->epRobotsMapping->setEp(i++, ep);
    }
    this->epRobotsMapping->sortEps();

    this->robotStateMapping = new StateCollection(this->epRobotsMapping);
    this->epSucMapping = std::make_shared<SuccessCollection>(p);
}

Assignment::Assignment(PartialAssignment* pa) {
    this->max = pa->getMax();
    this->min = max;
    this->plan = pa->getPlan();

    AssignmentCollection* assCol = pa->getEpRobotsMapping();
    if (AssignmentCollection::allowIdling) {
        this->epRobotsMapping = new AssignmentCollection(assCol->getSize() - 1);
    } else {
        this->epRobotsMapping = new AssignmentCollection(assCol->getSize());
    }

    for (short i = 0; i < this->epRobotsMapping->getSize(); i++) {
        // set the entrypoint
        epRobotsMapping->setEp(i, assCol->getEp(i));
        // copy robots
        *(epRobotsMapping->editRobots(i)) = *(assCol->getRobots(i));
    }

    this->robotStateMapping = new StateCollection(this->epRobotsMapping);
    this->epSucMapping = pa->getEpSuccessMapping();
}

Assignment::Assignment(const Plan* p, shared_ptr<AllocationAuthorityInfo> aai) {
    this->plan = p;
    this->max = 1;
    this->min = 1;

    this->epRobotsMapping = new AssignmentCollection(p->getEntryPoints().size());

    shared_ptr<vector<const supplementary::AgentID*>> curRobots;
    short i = 0;
    assert(p->getEntryPoints().size() == this->epRobotsMapping->getSize());
    for (const EntryPoint* ep : p->getEntryPoints()) {
        // set the entrypoint
        this->epRobotsMapping->setEp(i, ep);

        // curRobots = make_shared<vector<const supplementary::AgentID*>>();
        for (auto epRobots : aai->entryPointRobots) {
            // find the right entrypoint
            if (epRobots.entrypoint == ep->getId()) {
                // copy robots
                AgentSet* curRobots = this->epRobotsMapping->editRobots(i);

                for (auto& robot : epRobots.robots) {
                    curRobots->push_back(robot);
                }
                break;
            }
        }

        i++;
    }

    this->epSucMapping = std::make_shared<SuccessCollection>(p);
    this->robotStateMapping = new StateCollection(this->epRobotsMapping);
}

void Assignment::setPlan(const Plan* plan) {
    this->plan = plan;
}
StateCollection* Assignment::getRobotStateMapping() {
    return robotStateMapping;
}

AssignmentCollection* Assignment::getEpRobotsMapping() {
    return epRobotsMapping;
}

void Assignment::getAllRobots(AgentSet& o_robots) {
    for (int i = 0; i < this->epRobotsMapping->getSize(); ++i) {
        for (int j = 0; j < this->epRobotsMapping->getRobots(i)->size(); ++j) {
            o_robots.push_back(this->epRobotsMapping->getRobots(i)->at(j));
        }
    }
}

void Assignment::getAllRobotsSorted(AgentSet& o_robots) {
    getAllRobots(o_robots);
    sort(o_robots.begin(), o_robots.end(), supplementary::AgentIDComparator());
}

/**
 * The robots that are currently working on a specific task, referred to by an EntryPoint Id.
 * @param epid EntryPoint id
 * @return A vector of int
 */
const AgentSet* Assignment::getRobotsWorking(int64_t epid) const {
    return this->epRobotsMapping->getRobotsByEpId(epid);
}

void Assignment::getRobotsWorkingSorted(const EntryPoint* ep, AgentSet& o_robots) {
    o_robots = *getRobotsWorking(ep);
    sort(o_robots.begin(), o_robots.end(), supplementary::AgentIDComparator());
}

/**
 * The robots that are currently working on a specific task, referred to by an EntryPoint.
 * @param ep An EntryPoint
 * @return A vector of int
 */
const AgentSet* Assignment::getRobotsWorking(const EntryPoint* ep) const {
    return this->epRobotsMapping->getRobotsByEp(ep);
}

int Assignment::totalRobotCount() {
    int c = 0;
    for (int i = 0; i < this->epRobotsMapping->getSize(); i++) {
        c += this->epRobotsMapping->getRobots(i)->size();
    }
    return this->getNumUnAssignedRobotIds() + c;
}

//	/**
//	 * The shared_ptr of a vector of EntryPoints this assignment considers relevant.
//	 */
//	shared_ptr<vector<EntryPoint*> > Assignment::getEntryPoints()
//	{
//		return this->epRobotsMapping->getEntryPoints();
//	}

/**
 * Number of Entrypoints in this assignment's plan.
 */
short Assignment::getEntryPointCount() const {
    return this->epRobotsMapping->getSize();
}

/**
 * The robots that are currently working on or already succeeded in a specific task, referred to by an EntryPoint.
 * @param ep An EntryPoint
 * @return a shared_ptr of a list of int
 */
shared_ptr<list<const supplementary::AgentID*>> Assignment::getRobotsWorkingAndFinished(const EntryPoint* ep) {
    auto ret = std::make_shared<list<const supplementary::AgentID*>>(list<const supplementary::AgentID*>());
    auto robots = this->epRobotsMapping->getRobotsByEp(ep);
    if (robots != nullptr) {
        for (int i = 0; i < robots->size(); i++) {
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

/**
 * The robots that are currently working on or already succeeded in a specific task, referred to by an EntryPoint.
 * Each robot only occurs once.
 * @param ep An entrypoint
 * @return a shared_ptr of a list of int
 */
shared_ptr<list<const supplementary::AgentID*>> Assignment::getUniqueRobotsWorkingAndFinished(const EntryPoint* ep) {
    auto ret = std::make_shared<list<const supplementary::AgentID*>>(list<const supplementary::AgentID*>());
    // if (this->plan->getEntryPoints().find(ep->getId()) != this->plan->getEntryPoints().end())
    {
        auto robots = this->epRobotsMapping->getRobotsByEp(ep);
        for (int i = 0; i < robots->size(); i++) {
            ret->push_back(robots->at(i));
        }
        for (auto r : (*this->epSucMapping->getRobots(ep))) {
            if (std::find_if(ret->begin(), ret->end(), [&r](const supplementary::AgentID* id) { return *r == *id; }) ==
                    ret->end()) {
                ret->push_back(r);
            }
        }
    }
    return ret;
}

/**
 * The robots that are currently working on or already succeeded in a specific task, referred to by an EntryPoint Id.
 * @param epid EntryPoint id
 * @return a shared_ptr of a list of int
 */
std::shared_ptr<list<const supplementary::AgentID*>> Assignment::getRobotsWorkingAndFinished(int64_t epid) {
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

std::shared_ptr<SuccessCollection> Assignment::getEpSuccessMapping() {
    return this->epSucMapping;
}

void Assignment::setAllToInitialState(const AgentSet& robots, const EntryPoint* defep) {
    auto rlist = this->epRobotsMapping->editRobotsByEp(defep);
    for (const supplementary::AgentID* r : robots) {
        rlist->push_back(r);
        robotStateMapping->setState(r, defep->getState());
    }
}

bool Assignment::removeRobot(const supplementary::AgentID* robotId) {
    this->robotStateMapping->removeRobot(robotId);
    AgentSet* curRobots;
    for (int i = 0; i < this->epRobotsMapping->getSize(); ++i) {
        curRobots = this->epRobotsMapping->editRobots(i);
        auto iter = find_if(curRobots->begin(), curRobots->end(),
                [&robotId](const supplementary::AgentID* id) { return *robotId == *id; });
        if (iter != curRobots->end()) {
            curRobots->erase(iter);
            return true;
        }
    }
    return false;
}

void Assignment::addRobot(const supplementary::AgentID* id, const EntryPoint* e, const State* s) {
    if (e == nullptr) {
        return;
    }
    this->robotStateMapping->setState(id, s);
    this->epRobotsMapping->addRobot(id, e);
    return;
}

/**
 * Tests whether this assignment is valid with respect to the plan's cardinalities.
 * @return A bool
 */
bool Assignment::isValid() const {
    const vector<shared_ptr<std::list<const supplementary::AgentID*>>>& success = this->epSucMapping->getRobots();

    for (int i = 0; i < this->epRobotsMapping->getSize(); ++i) {
        int c = this->epRobotsMapping->getRobots(i)->size() + success[i]->size();
        if (c > this->epRobotsMapping->getEp(i)->getMaxCardinality() ||
                c < this->epRobotsMapping->getEp(i)->getMinCardinality()) {
            return false;
        }
    }
    return true;
}

/**
 * Tests weather all required tasks have been successfully completed and
 * thus the plan can be considered as successful.
 * @return A bool
 */
bool Assignment::isSuccessfull() const {
    for (int i = 0; i < this->epSucMapping->getCount(); i++) {
        if (this->epSucMapping->getEntryPoints()[i]->isSuccessRequired()) {
            if (!(this->epSucMapping->getRobots()[i]->size() > 0 &&
                        static_cast<int>(this->epSucMapping->getRobots()[i]->size()) >=
                                this->epSucMapping->getEntryPoints()[i]->getMinCardinality())) {
                return false;
            }
        }
    }
    return true;
}

bool Assignment::isEqual(Assignment* otherAssignment) {
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
            auto iter = find_if(otherAssignment->epRobotsMapping->getRobots(i)->begin(),
                    otherAssignment->epRobotsMapping->getRobots(i)->end(),
                    [&robot](const supplementary::AgentID* id) { return *robot == *id; });
            if (iter == otherAssignment->epRobotsMapping->getRobots(i)->end()) {
                return false;
            }
        }
    }
    return true;
}

/**
 * Test whether at least one robot is working on a task or succeeded with a task.
 * @param ep An EntryPoint identifying the task in question.
 * @return bool
 */
bool Assignment::isEntryPointNonEmpty(const EntryPoint* ep) const {
    auto r = this->epRobotsMapping->getRobotsByEp(ep);
    if (r != nullptr && r->size() > 0) {
        return true;
    }
    auto epSuc = this->epSucMapping->getRobots(ep);
    return (epSuc != nullptr && epSuc->size() > 0);
}

bool Assignment::updateRobot(const supplementary::AgentID* robot, const EntryPoint* ep, const State* s) {
    this->robotStateMapping->setState(robot, s);
    bool ret = false;

    for (int i = 0; i < this->epRobotsMapping->getSize(); i++) {
        if (this->epRobotsMapping->getEp(i) == ep) {
            if (find_if(this->epRobotsMapping->getRobots(i)->begin(), this->epRobotsMapping->getRobots(i)->end(),
                        [&robot](const supplementary::AgentID* id) { return *robot == *id; }) !=
                    this->epRobotsMapping->getRobots(i)->end()) {
                return false;
            } else {
                this->epRobotsMapping->editRobots(i)->push_back(robot);
                ret = true;
            }
        } else {
            auto iter =
                    find_if(this->epRobotsMapping->getRobots(i)->begin(), this->epRobotsMapping->getRobots(i)->end(),
                            [&robot](const supplementary::AgentID* id) { return *robot == *id; });
            if (iter != this->epRobotsMapping->getRobots(i)->end()) {
                this->epRobotsMapping->editRobots(i)->erase(iter);
                ret = true;
            }
        }
    }
    return ret;
}

bool Assignment::updateRobot(const supplementary::AgentID* robot, const EntryPoint* ep) {
    bool ret = false;
    for (int i = 0; i < this->epRobotsMapping->getSize(); i++) {
        if (this->epRobotsMapping->getEp(i) == ep) {
            if (find_if(this->epRobotsMapping->getRobots(i)->begin(), this->epRobotsMapping->getRobots(i)->end(),
                        [&robot](const supplementary::AgentID* id) { return *robot == *id; }) !=
                    this->epRobotsMapping->getRobots(i)->end()) {
                return false;
            } else {
                this->epRobotsMapping->editRobots(i)->push_back(robot);
                ret = true;
            }
        } else {
            auto iter =
                    find_if(this->epRobotsMapping->getRobots(i)->begin(), this->epRobotsMapping->getRobots(i)->end(),
                            [&robot](const supplementary::AgentID* id) { return *robot == *id; });
            if (iter != this->epRobotsMapping->getRobots(i)->end()) {
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

bool Assignment::removeRobot(const supplementary::AgentID* robot, const EntryPoint* ep) {
    assert(ep != nullptr);
    if (ep == nullptr) {
        return false;
    }
    this->robotStateMapping->removeRobot(robot);
    return epRobotsMapping->removeRobot(robot, ep);
}

std::string Assignment::assignmentCollectionToString() {
    std::stringstream ss;
    ss << "ASS" << std::endl;
    ss << this->epRobotsMapping->toString();
    return ss.str();
}

void Assignment::addRobot(const supplementary::AgentID* id, const EntryPoint* e) {
    assert(e != nullptr);
    if (e == nullptr) {
        return;
    }
    this->epRobotsMapping->addRobot(id, e);
    return;
}

void Assignment::moveRobots(const State* from, const State* to) {
    assert(to != nullptr);
    robotStateMapping->moveAllFromTo(from, to);
}

/**
 * Returns the EntryPoint a robot is currently working on. Returns null, if the robot is currently not working on the
 * respective plan.
 * @param robot an int
 * @return An entrypoint
 */
const EntryPoint* Assignment::getEntryPointOfRobot(const supplementary::AgentID* robot) {
    for (int i = 0; i < this->epRobotsMapping->getSize(); i++) {
        auto iter = find_if(this->epRobotsMapping->getRobots(i)->begin(), this->epRobotsMapping->getRobots(i)->end(),
                [&robot](const supplementary::AgentID* id) { return *robot == *id; });
        if (iter != this->epRobotsMapping->getRobots(i)->end()) {
            return this->epRobotsMapping->getEp(i);
        }
    }
    return nullptr;
}

void Assignment::clear() {
    this->robotStateMapping->clear();
    this->epRobotsMapping->clear();
    this->epSucMapping->clear();
}

std::string Assignment::toString() {
    std::stringstream ss;
    ss << std::endl;
    ss << "Rating: " << this->max << std::endl;
    for (int i = 0; i < this->epRobotsMapping->getSize(); ++i) {
        ss << "EP: " << this->epRobotsMapping->getEp(i)->getId()
           << " Task: " << this->epRobotsMapping->getEp(i)->getTask()->getName() << " RobotIDs: ";
        for (const supplementary::AgentID* robot : *(this->epRobotsMapping->getRobots(i))) {
            ss << *robot << " ";
        }
        ss << std::endl;
    }
    ss << "Robot-State Mapping:\n";
    ss << this->robotStateMapping->toString();
    ss << this->epSucMapping->toString() << std::endl;
    return ss.str();
}

std::string Assignment::toHackString() {
    std::stringstream ss;
    ss << "ASS " << this->plan->getId() << " " << this->plan->getName() << ":\t";
    auto suc = this->epSucMapping->getRobots();
    for (int i = 0; i < this->epRobotsMapping->getSize(); i++) {
        ss << this->epRobotsMapping->getEp(i)->getTask()->getName() << " ";
        for (auto& robotID : *(this->epRobotsMapping->getRobots(i))) {
            ss << *robotID << string(" ");
        }

        if (suc[i]->size() > 0) {
            ss << "\t Success: ";
            for (auto& robotID : (*suc[i])) {
                ss << *robotID << string(" ");
            }
        }
    }
    ss << std::endl;
    return ss.str();
}

} /* namespace alica */
