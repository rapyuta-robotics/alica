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

namespace alica {

Assignment::~Assignment() {
    delete this->epRobotsMapping;
    delete this->robotStateMapping;
}
Assignment::Assignment(Plan* p) {
    this->plan = p;
    this->max = 0.0;
    this->min = 0.0;

    this->epRobotsMapping = new AssignmentCollection(this->plan->getEntryPoints().size());

    // sort the entrypoints of the given plan
    list<EntryPoint*> sortedEpList;
    for (auto pair : plan->getEntryPoints()) {
        sortedEpList.push_back(pair.second);
    }
    sortedEpList.sort(EntryPoint::compareTo);

    // add the sorted entrypoints into the assignmentcollection
    short i = 0;
    for (EntryPoint* ep : sortedEpList) {
        this->epRobotsMapping->setEp(i++, ep);
    }
    this->epRobotsMapping->sortEps();

    this->robotStateMapping = new StateCollection(this->epRobotsMapping);
    this->epSucMapping = make_shared<SuccessCollection>(p);
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

    shared_ptr<vector<const supplementary::AgentID*>> curRobots;
    for (short i = 0; i < this->epRobotsMapping->getSize(); i++) {
        // set the entrypoint
        if (!this->epRobotsMapping->setEp(i, assCol->getEp(i))) {
            cerr << "Ass: AssignmentCollection Index out of entrypoints bounds!" << endl;
            throw new exception();
        }

        // copy robots
        auto robots = assCol->getRobots(i);
        curRobots = make_shared<vector<const supplementary::AgentID*>>();
        for (auto& rob : *robots) {
            curRobots->push_back(rob);
        }

        // set the robots
        if (!this->epRobotsMapping->setRobots(i, curRobots)) {
            cerr << "Ass: AssignmentCollection Index out of robots bounds!" << endl;
            throw new exception();
        }
    }

    this->robotStateMapping = new StateCollection(this->epRobotsMapping);
    this->epSucMapping = pa->getEpSuccessMapping();
}

Assignment::Assignment(Plan* p, shared_ptr<AllocationAuthorityInfo> aai) {
    this->plan = p;
    this->max = 1;
    this->min = 1;

    this->epRobotsMapping = new AssignmentCollection(p->getEntryPoints().size());

    shared_ptr<vector<const supplementary::AgentID*>> curRobots;
    short i = 0;
    for (auto epPair : p->getEntryPoints()) {
        // set the entrypoint
        if (!this->epRobotsMapping->setEp(i, epPair.second)) {
            cerr << "Ass: AssignmentCollection Index out of entrypoints bounds!" << endl;
            throw new exception();
        }

        curRobots = make_shared<vector<const supplementary::AgentID*>>();
        for (auto epRobots : aai->entryPointRobots) {
            // find the right entrypoint
            if (epRobots.entrypoint == epPair.second->getId()) {
                // copy robots
                for (auto& robot : epRobots.robots) {
                    curRobots->push_back(robot);
                }

                // set the robots
                if (!this->epRobotsMapping->setRobots(i, curRobots)) {
                    cerr << "Ass: AssignmentCollection Index out of robots bounds!" << endl;
                    throw new exception();
                }

                break;
            }
        }

        i++;
    }

    this->epSucMapping = make_shared<SuccessCollection>(p);
    this->robotStateMapping = new StateCollection(this->epRobotsMapping);
}

Plan* Assignment::getPlan() {
    return plan;
}

void Assignment::setPlan(Plan* plan) {
    this->plan = plan;
}
StateCollection* Assignment::getRobotStateMapping() {
    return robotStateMapping;
}

shared_ptr<vector<const supplementary::AgentID*>> Assignment::getAllRobotsSorted() {
    shared_ptr<vector<const supplementary::AgentID*>> ret = shared_ptr<vector<const supplementary::AgentID*>>();
    for (int i = 0; i < this->getEpRobotsMapping()->getSize(); i++) {
        for (int j = 0; j < this->getEpRobotsMapping()->getRobots(i)->size(); j++) {
            ret->push_back(this->getEpRobotsMapping()->getRobots(i)->at(j));
        }
    }
    sort(ret->begin(), ret->end());
    return ret;
}

AssignmentCollection* Assignment::getEpRobotsMapping() {
    return epRobotsMapping;
}

/**
 * The robots that are currently working on a specific task, referred to by an EntryPoint Id.
 * @param epid EntryPoint id
 * @return A vector of int
 */
shared_ptr<vector<const supplementary::AgentID*>> Assignment::getRobotsWorking(long epid) {
    return this->getEpRobotsMapping()->getRobotsByEpId(epid);
}

shared_ptr<vector<const supplementary::AgentID*>> Assignment::getRobotsWorkingSorted(EntryPoint* ep) {
    auto ret = getRobotsWorking(ep);
    sort(ret->begin(), ret->end());
    return ret;
}

/**
 * The robots that are currently working on a specific task, referred to by an EntryPoint.
 * @param ep An EntryPoint
 * @return A vector of int
 */
shared_ptr<vector<const supplementary::AgentID*>> Assignment::getRobotsWorking(EntryPoint* ep) {
    return this->getEpRobotsMapping()->getRobotsByEp(ep);
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
short Assignment::getEntryPointCount() {
    return this->epRobotsMapping->getSize();
}

/**
 * The robots that are currently working on or already succeeded in a specific task, referred to by an EntryPoint.
 * @param ep An EntryPoint
 * @return a shared_ptr of a list of int
 */
shared_ptr<list<const supplementary::AgentID*>> Assignment::getRobotsWorkingAndFinished(EntryPoint* ep) {
    auto ret = make_shared<list<const supplementary::AgentID*>>(list<const supplementary::AgentID*>());
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
shared_ptr<list<const supplementary::AgentID*>> Assignment::getUniqueRobotsWorkingAndFinished(EntryPoint* ep) {
    auto ret = make_shared<list<const supplementary::AgentID*>>(list<const supplementary::AgentID*>());
    // if (this->plan->getEntryPoints().find(ep->getId()) != this->plan->getEntryPoints().end())
    {
        auto robots = this->epRobotsMapping->getRobotsByEp(ep);
        for (int i = 0; i < robots->size(); i++) {
            ret->push_back(robots->at(i));
        }
        for (auto r : (*this->epSucMapping->getRobots(ep))) {
            if (find_if(ret->begin(), ret->end(), [&r](const supplementary::AgentID* id) { return *r == *id; }) ==
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
shared_ptr<list<const supplementary::AgentID*>> Assignment::getRobotsWorkingAndFinished(long epid) {
    auto ret = make_shared<list<const supplementary::AgentID*>>(list<const supplementary::AgentID*>());
    auto robots = this->epRobotsMapping->getRobotsByEpId(epid);
    if (robots != nullptr) {
        for (int i = 0; i < robots->size(); i++) {
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

shared_ptr<SuccessCollection> Assignment::getEpSuccessMapping() {
    return this->epSucMapping;
}

void Assignment::setAllToInitialState(const std::vector<const supplementary::AgentID*>& robots, EntryPoint* defep) {
    auto rlist = this->epRobotsMapping->getRobotsByEp(defep);
    for (const supplementary::AgentID* r : robots) {
        rlist->push_back(r);
        robotStateMapping->setState(r, defep->getState());
    }
}

bool Assignment::removeRobot(const supplementary::AgentID* robotId) {
    this->robotStateMapping->removeRobot(robotId);
    shared_ptr<vector<const supplementary::AgentID*>> curRobots;
    for (int i = 0; i < this->epRobotsMapping->getSize(); i++) {
        curRobots = this->epRobotsMapping->getRobots(i);
        auto iter = find_if(curRobots->begin(), curRobots->end(),
                [&robotId](const supplementary::AgentID* id) { return *robotId == *id; });
        if (iter != curRobots->end()) {
            curRobots->erase(iter);
            return true;
        }
    }
    return false;
}

void Assignment::addRobot(const supplementary::AgentID* id, EntryPoint* e, State* s) {
    if (e == nullptr) {
        return;
    }
    this->robotStateMapping->setState(id, s);
    this->epRobotsMapping->getRobotsByEp(e)->push_back(id);
    return;
}

/**
 * Tests whether this assignment is valid with respect to the plan's cardinalities.
 * @return A bool
 */
bool Assignment::isValid() {
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
bool Assignment::isSuccessfull() {
    for (int i = 0; i < this->epSucMapping->getCount(); i++) {
        if (this->epSucMapping->getEntryPoints()[i]->getSuccessRequired()) {
            if (!(this->epSucMapping->getRobots()[i]->size() > 0 &&
                        this->epSucMapping->getRobots()[i]->size() >=
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
bool Assignment::isEntryPointNonEmpty(EntryPoint* ep) {
    auto r = this->epRobotsMapping->getRobotsByEp(ep);
    if (r != nullptr && r->size() > 0) {
        return true;
    }
    auto epSuc = this->epSucMapping->getRobots(ep);
    return (epSuc != nullptr && epSuc->size() > 0);
}

bool Assignment::updateRobot(const supplementary::AgentID* robot, EntryPoint* ep, State* s) {
    this->robotStateMapping->setState(robot, s);
    bool ret = false;

    for (int i = 0; i < this->epRobotsMapping->getSize(); i++) {
        if (this->epRobotsMapping->getEp(i) == ep) {
            if (find_if(this->epRobotsMapping->getRobots(i)->begin(), this->epRobotsMapping->getRobots(i)->end(),
                        [&robot](const supplementary::AgentID* id) { return *robot == *id; }) !=
                    this->epRobotsMapping->getRobots(i)->end()) {
                return false;
            } else {
                this->epRobotsMapping->getRobots(i)->push_back(robot);
                ret = true;
            }
        } else {
            auto iter =
                    find_if(this->epRobotsMapping->getRobots(i)->begin(), this->epRobotsMapping->getRobots(i)->end(),
                            [&robot](const supplementary::AgentID* id) { return *robot == *id; });
            if (iter != this->epRobotsMapping->getRobots(i)->end()) {
                this->epRobotsMapping->getRobots(i)->erase(iter);
                ret = true;
            }
        }
    }
    return ret;
}

bool Assignment::updateRobot(const supplementary::AgentID* robot, EntryPoint* ep) {
    bool ret = false;
    for (int i = 0; i < this->epRobotsMapping->getSize(); i++) {
        if (this->epRobotsMapping->getEp(i) == ep) {
            if (find_if(this->epRobotsMapping->getRobots(i)->begin(), this->epRobotsMapping->getRobots(i)->end(),
                        [&robot](const supplementary::AgentID* id) { return *robot == *id; }) !=
                    this->epRobotsMapping->getRobots(i)->end()) {
                return false;
            } else {
                this->epRobotsMapping->getRobots(i)->push_back(robot);
                ret = true;
            }
        } else {
            auto iter =
                    find_if(this->epRobotsMapping->getRobots(i)->begin(), this->epRobotsMapping->getRobots(i)->end(),
                            [&robot](const supplementary::AgentID* id) { return *robot == *id; });
            if (iter != this->epRobotsMapping->getRobots(i)->end()) {
                this->epRobotsMapping->getRobots(i)->erase(iter);
                ret = true;
            }
        }
    }
    if (ret) {
        this->robotStateMapping->setState(robot, ep->getState());
    }
    return ret;
}

bool Assignment::removeRobot(const supplementary::AgentID* robot, EntryPoint* ep) {
    if (ep == nullptr) {
        return false;
    }
    this->robotStateMapping->removeRobot(robot);
    auto iter =
            find_if(this->epRobotsMapping->getRobotsByEp(ep)->begin(), this->epRobotsMapping->getRobotsByEp(ep)->end(),
                    [&robot](const supplementary::AgentID* id) { return *robot == *id; });
    if (iter != this->epRobotsMapping->getRobotsByEp(ep)->end()) {
        this->epRobotsMapping->getRobotsByEp(ep)->erase(iter);
        return true;
    } else {
        return false;
    }
}

string Assignment::assignmentCollectionToString() {
    stringstream ss;
    ss << "ASS" << endl;
    ss << this->epRobotsMapping->toString();
    return ss.str();
}

void Assignment::addRobot(const supplementary::AgentID* id, EntryPoint* e) {
    if (e == nullptr) {
        return;
    }
    this->epRobotsMapping->getRobotsByEp(e)->push_back(id);
    return;
}

void Assignment::moveRobots(State* from, State* to) {
    auto movingRobots = this->robotStateMapping->getRobotsInState(from);
    if (to == nullptr) {
        cout << "Ass: MoveRobots is given a State which is NULL!" << endl;
    }
    for (auto& r : movingRobots) {
        this->robotStateMapping->setState(r, to);
    }
}

/**
 * Returns the EntryPoint a robot is currently working on. Returns null, if the robot is currently not working on the
 * respective plan.
 * @param robot an int
 * @return An entrypoint
 */
EntryPoint* Assignment::getEntryPointOfRobot(const supplementary::AgentID* robot) {
    for (int i = 0; i < this->epRobotsMapping->getSize(); i++) {
        auto iter = find_if(this->epRobotsMapping->getRobots(i)->begin(), this->epRobotsMapping->getRobots(i)->end(),
                [&robot](const supplementary::AgentID* id) { return *robot == *id; });
        if (iter != this->epRobotsMapping->getRobots(i)->end()) {
            return this->epRobotsMapping->getEp(i);
        }
    }
    return nullptr;
}

/**
 * The list of all robots currently allocated by this assignment to work on any task within the plan.
 * @return A shared_ptr of a vector of int
 */
shared_ptr<vector<const supplementary::AgentID*>> Assignment::getAllRobots() {
    auto ret = make_shared<vector<const supplementary::AgentID*>>(vector<const supplementary::AgentID*>());
    for (int i = 0; i < this->epRobotsMapping->getSize(); i++) {
        for (int j = 0; j < this->epRobotsMapping->getRobots(i)->size(); j++) {
            ret->push_back(this->epRobotsMapping->getRobots(i)->at(j));
        }
    }
    return ret;
}

void Assignment::clear() {
    this->robotStateMapping->clear();
    this->epRobotsMapping->clear();
    this->epSucMapping->clear();
}

string Assignment::toString() {
    stringstream ss;
    ss << endl;
    ss << "Rating: " << this->max << endl;
    for (int i = 0; i < this->epRobotsMapping->getSize(); ++i) {
        ss << "EP: " << this->epRobotsMapping->getEp(i)->getId()
           << " Task: " << this->epRobotsMapping->getEp(i)->getTask()->getName() << " RobotIDs: ";
        for (const supplementary::AgentID* robot : *(this->epRobotsMapping->getRobots(i))) {
            ss << *robot << " ";
        }
        ss << endl;
    }
    ss << "Robot-State Mapping:\n";
    ss << this->robotStateMapping->toString();
    ss << this->epSucMapping->toString() << endl;
    return ss.str();
}

string Assignment::toHackString() {
    stringstream ss;
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
    ss << endl;
    return ss.str();
}

} /* namespace alica */
