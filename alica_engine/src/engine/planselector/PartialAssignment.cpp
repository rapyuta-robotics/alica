#include <engine/planselector/PartialAssignment.h>
#include <engine/planselector/PartialAssignmentPool.h>

#include "engine/SimplePlanTree.h"
#include "engine/collections/AssignmentCollection.h"
#include "engine/collections/SuccessCollection.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Plan.h"
#include "engine/model/Task.h"
#include "engine/planselector/DynCardinality.h"

namespace alica {

int PartialAssignment::getHash() {
    if (hashCalculated) {
        return hash;
    } else {
        std::hash<alica::PartialAssignment> paHash;
        return paHash(*this);
    }
}

void PartialAssignment::setHash(int hash = 0) {
    this->hash = hash;
}

bool PartialAssignment::isHashCalculated() {
    return hashCalculated;
}

void PartialAssignment::setHashCalculated(bool hashCalculated) {
    this->hashCalculated = hashCalculated;
}

PartialAssignment::PartialAssignment(PartialAssignmentPool* pap) {
    this->pap = pap;
    this->utilFunc = nullptr;
    this->epSuccessMapping = nullptr;
    this->hashCalculated = false;
    this->plan = nullptr;
    this->epRobotsMapping = new AssignmentCollection(AssignmentCollection::maxEpsCount);
    this->unassignedRobotIds = vector<const supplementary::AgentID*>();
    this->dynCardinalities = vector<shared_ptr<DynCardinality>>(AssignmentCollection::maxEpsCount);
    this->compareVal = PRECISION;
    for (int i = 0; i < AssignmentCollection::maxEpsCount; i++) {
        this->dynCardinalities[i] = make_shared<DynCardinality>();
    }
}

PartialAssignment::~PartialAssignment() {
    delete epRobotsMapping;
}

AssignmentCollection* PartialAssignment::getEpRobotsMapping() {
    return epRobotsMapping;
}

Plan* PartialAssignment::getPlan() {
    return plan;
}

shared_ptr<UtilityFunction> PartialAssignment::getUtilFunc() {
    return utilFunc;
}

shared_ptr<SuccessCollection> PartialAssignment::getEpSuccessMapping() {
    return epSuccessMapping;
}

//	vector<int>& PartialAssignment::getunassignedRobots()
//	{
//		return unassignedRobots;
//	}

void PartialAssignment::clear() {
    this->min = 0.0;
    this->max = 0.0;
    this->compareVal = PRECISION;
    this->unassignedRobotIds.clear();
    for (int i = 0; i < this->epRobotsMapping->getSize(); i++) {
        this->epRobotsMapping->getRobots(i)->clear();
    }
    this->hashCalculated = false;
}

void PartialAssignment::reset(PartialAssignmentPool* pap) {
    pap->curIndex = 0;
}

shared_ptr<vector<const supplementary::AgentID*>> PartialAssignment::getRobotIds() {
    return this->robotIds;
}

PartialAssignment* PartialAssignment::getNew(PartialAssignmentPool* pap,
        shared_ptr<vector<const supplementary::AgentID*>> robotIds, Plan* plan, shared_ptr<SuccessCollection> sucCol) {
    if (pap->curIndex >= pap->maxCount) {
        cerr << "max PA count reached!" << endl;
    }
    PartialAssignment* ret = pap->daPAs[pap->curIndex++];
    ret->clear();
    ret->robotIds = robotIds;  // Should already be sorted! (look at TaskAssignment, or PlanSelector)
    ret->plan = plan;
    ret->utilFunc = plan->getUtilityFunction();
    ret->epSuccessMapping = sucCol;
    // Create EP-Array
    if (AssignmentCollection::allowIdling) {
        ret->epRobotsMapping->setSize(plan->getEntryPoints().size() + 1);
        // Insert IDLE-EntryPoint
        ret->epRobotsMapping->setEp(ret->epRobotsMapping->getSize() - 1, pap->idleEP);
    } else {
        ret->epRobotsMapping->setSize(plan->getEntryPoints().size());
    }
    // Insert plan entrypoints
    int i = 0;
    for (auto iter : plan->getEntryPoints()) {
        ret->epRobotsMapping->setEp(i++, iter.second);
    }

    // Sort the entrypoint array
    ret->epRobotsMapping->sortEps();

    for (int i = 0; i < ret->epRobotsMapping->getSize(); i++) {
        ret->dynCardinalities[i]->setMin(ret->epRobotsMapping->getEp(i)->getMinCardinality());
        ret->dynCardinalities[i]->setMax(ret->epRobotsMapping->getEp(i)->getMaxCardinality());
        shared_ptr<list<const supplementary::AgentID*>> suc = sucCol->getRobots(ret->epRobotsMapping->getEp(i));

        if (suc != nullptr) {
            ret->dynCardinalities[i]->setMin(ret->dynCardinalities[i]->getMin() - suc->size());
            ret->dynCardinalities[i]->setMax(ret->dynCardinalities[i]->getMax() - suc->size());
            if (ret->dynCardinalities[i]->getMin() < 0) {
                ret->dynCardinalities[i]->setMin(0);
            }
            if (ret->dynCardinalities[i]->getMax() < 0) {
                ret->dynCardinalities[i]->setMax(0);
            }

#ifdef SUCDEBUG
            cout << "SuccessCollection" << endl;
            cout << "EntryPoint: " << ret->epRobotsMapping->getEntryPoints()->at(i)->toString() << endl;
            cout << "DynMax: " << ret->dynCardinalities[i]->getMax() << endl;
            cout << "DynMin: " << ret->dynCardinalities[i]->getMin() << endl;
            cout << "SucCol: ";
            for (int j : (*suc)) {
                cout << j << ", ";
            }
            cout << "-----------" << endl;
#endif
        }
    }

    // At the beginning all robots are unassigned
    for (auto& robotId : (*robotIds)) {
        ret->unassignedRobotIds.push_back(robotId);
    }
    return ret;
}

PartialAssignment* PartialAssignment::getNew(PartialAssignmentPool* pap, PartialAssignment* oldPA) {
    if (pap->curIndex >= pap->maxCount) {
        cerr << "max PA count reached!" << endl;
    }
    PartialAssignment* ret = pap->daPAs[pap->curIndex++];
    ret->clear();
    ret->min = oldPA->min;
    ret->max = oldPA->max;
    ret->plan = oldPA->plan;
    ret->robotIds = oldPA->robotIds;
    ret->utilFunc = oldPA->utilFunc;
    ret->epSuccessMapping = oldPA->epSuccessMapping;
    for (int i = 0; i < oldPA->unassignedRobotIds.size(); i++) {
        ret->unassignedRobotIds.push_back(oldPA->unassignedRobotIds[i]);
    }

    for (int i = 0; i < oldPA->dynCardinalities.size(); i++) {
        ret->dynCardinalities[i] =
                make_shared<DynCardinality>(oldPA->dynCardinalities[i]->getMin(), oldPA->dynCardinalities[i]->getMax());
    }

    ret->epRobotsMapping->setSize(oldPA->epRobotsMapping->getSize());

    for (int i = 0; i < oldPA->epRobotsMapping->getSize(); i++) {
        ret->epRobotsMapping->setEp(i, oldPA->epRobotsMapping->getEp(i));
        for (int j = 0; j < oldPA->epRobotsMapping->getRobots(i)->size(); j++) {
            ret->epRobotsMapping->getRobots(i)->push_back(oldPA->epRobotsMapping->getRobots(i)->at(j));
        }
    }

    return ret;
}

short PartialAssignment::getEntryPointCount() {
    return this->epRobotsMapping->getSize();
}

int PartialAssignment::totalRobotCount() {
    int c = 0;
    for (int i = 0; i < this->epRobotsMapping->getSize(); i++) {
        c += this->epRobotsMapping->getRobots(i)->size();
    }
    return this->getNumUnAssignedRobotIds() + c;
}

shared_ptr<vector<const supplementary::AgentID*>> PartialAssignment::getRobotsWorking(EntryPoint* ep) {
    return this->epRobotsMapping->getRobotsByEp(ep);
}

shared_ptr<vector<const supplementary::AgentID*>> PartialAssignment::getRobotsWorking(long epid) {
    return this->epRobotsMapping->getRobotsByEpId(epid);
}

shared_ptr<list<const supplementary::AgentID*>> PartialAssignment::getRobotsWorkingAndFinished(EntryPoint* ep) {
    shared_ptr<list<const supplementary::AgentID*>> ret =
            make_shared<list<const supplementary::AgentID*>>(list<const supplementary::AgentID*>());
    auto robotIds = this->epRobotsMapping->getRobotsByEp(ep);
    if (robotIds != nullptr) {
        for (auto iter : (*robotIds)) {
            ret->push_back(iter);
        }
    }
    auto successes = this->epSuccessMapping->getRobots(ep);
    if (successes != nullptr) {
        for (auto iter : (*successes)) {
            ret->push_back(iter);
        }
    }
    return ret;
}

shared_ptr<list<const supplementary::AgentID*>> PartialAssignment::getRobotsWorkingAndFinished(long epid) {
    shared_ptr<list<const supplementary::AgentID*>> ret =
            make_shared<list<const supplementary::AgentID*>>(list<const supplementary::AgentID*>());
    auto robots = this->epRobotsMapping->getRobotsByEpId(epid);
    if (robots != nullptr) {
        for (auto iter : (*robots)) {
            ret->push_back(iter);
        }
    }
    auto successes = this->epSuccessMapping->getRobotsById(epid);
    if (successes != nullptr) {
        for (auto iter : (*successes)) {
            ret->push_back(iter);
        }
    }
    return ret;
}

shared_ptr<list<const supplementary::AgentID*>> PartialAssignment::getUniqueRobotsWorkingAndFinished(EntryPoint* ep) {
    auto ret = make_shared<list<const supplementary::AgentID*>>(list<const supplementary::AgentID*>());
    auto robots = this->epRobotsMapping->getRobotsByEp(ep);

    for (auto iter : (*robots)) {
        ret->push_back(iter);
    }

    auto successes = this->epSuccessMapping->getRobots(ep);
    if (successes != nullptr) {
        for (auto iter : (*successes)) {
            if (find_if(ret->begin(), ret->end(), [&iter](const supplementary::AgentID* id) { return *iter == *id; }) ==
                    ret->end()) {
                ret->push_back(iter);
            }
        }
    }
    return ret;
}

/**
 * If the robot has already assigned itself, this method updates the partial assignment accordingly
 * @param A shared_ptr<SimplePlanTree>
 * @param An int
 * @return A bool
 */
bool PartialAssignment::addIfAlreadyAssigned(shared_ptr<SimplePlanTree> spt, const supplementary::AgentID* robotId) {
    if (spt->getEntryPoint()->getPlan() == this->plan) {
        EntryPoint* curEp;
        int max = this->epRobotsMapping->getSize();
        if (AssignmentCollection::allowIdling) {
            max--;
        }
        for (int i = 0; i < max; ++i) {
            curEp = this->epRobotsMapping->getEp(i);
            if (spt->getEntryPoint()->getId() == curEp->getId()) {
                if (!this->assignRobot(robotId, i)) {
                    break;
                }
                // remove robot from "To-Add-List"
                auto iter = find_if(this->unassignedRobotIds.begin(), this->unassignedRobotIds.end(),
                        [&robotId](const supplementary::AgentID* id) { return *robotId == *id; });
                if (this->unassignedRobotIds.erase(iter) == this->unassignedRobotIds.end()) {
                    cerr << "PA: Tried to assign robot " << robotId << ", but it was NOT UNassigned!" << endl;
                    throw new exception;
                }
                // return true, because we are ready, when we found the robot here
                return true;
            }
        }
        return false;
    }
    // If there are children and we didnt find the robot until now, then go on recursive
    else if (spt->getChildren().size() > 0) {
        for (auto sptChild : spt->getChildren()) {
            if (this->addIfAlreadyAssigned(sptChild, robotId)) {
                return true;
            }
        }
    }
    // Did not find the robot in any relevant entry point
    return false;
}

/**
 * Assigns the robot into the data structures according to the given index.
 * @return True, when it was possible to assign the robot. False, otherwise.
 */
bool PartialAssignment::assignRobot(const supplementary::AgentID* robotId, int index) {
    if (this->dynCardinalities[index]->getMax() > 0) {
        this->epRobotsMapping->getRobots(index)->push_back(robotId);
        if (this->dynCardinalities[index]->getMin() > 0) {
            this->dynCardinalities[index]->setMin(this->dynCardinalities[index]->getMin() - 1);
        }
        if (this->dynCardinalities[index]->getMax() != INFINITY) {
            this->dynCardinalities[index]->setMax(this->dynCardinalities[index]->getMax() - 1);
        }
        return true;
    }
    return false;
}

shared_ptr<list<PartialAssignment*>> PartialAssignment::expand() {
    shared_ptr<list<PartialAssignment*>> newPas = make_shared<list<PartialAssignment*>>();
    if (this->unassignedRobotIds.size() == 0) {
        // No robot left to expand
        return newPas;
    }
    // Robot which should be assigned next
    const supplementary::AgentID* robot = this->unassignedRobotIds[0];
    this->unassignedRobotIds.erase(this->unassignedRobotIds.begin());
    PartialAssignment* newPa;
    for (int i = 0; i < this->epRobotsMapping->getSize(); ++i) {
        if (this->dynCardinalities[i]->getMax() > 0) {
            // Update the cardinalities and assign the robot
            newPa = PartialAssignment::getNew(pap, this);
            newPa->assignRobot(robot, i);
            newPas->push_back(newPa);
        }
    }
    return newPas;
}

/**
 * Checks whether the current assignment is valid
 */
bool PartialAssignment::isValid() {
    int min = 0;
    for (int i = 0; i < this->epRobotsMapping->getSize(); ++i) {
        min += dynCardinalities[i]->getMin();
    }
    return min <= this->getNumUnAssignedRobotIds();
}

/**
 * Checks if this PartialAssignment is a complete Assignment.
 * @return True, if it is, false otherwise.
 */
bool PartialAssignment::isGoal() {
    // There should be no unassigned robots anymore
    if (this->unassignedRobotIds.size() > 0) {
        return false;
    }
    // Every EntryPoint should be satisfied according to his minCar
    for (int i = 0; i < this->epRobotsMapping->getSize(); ++i) {
        if (this->dynCardinalities[i]->getMin() != 0) {
            return false;
        }
    }
    return true;
}

/**
 * Compares this PartialAssignment with another one.
 * @return false if it is the same object or they have the same utility, assignment and plan id
 * false if this PartialAssignment has a higher utility, or plan id
 * Difference between Hashcodes, if they have the same utility and plan id
 * true if the other PartialAssignment has a higher utility, or plan id
 */
bool PartialAssignment::compareTo(PartialAssignment* thisPa, PartialAssignment* newPa) {
    // TODO has perhaps to be changed
    // 0 , -1 = false
    // 1 true
    if (&thisPa == &newPa)  // Same reference -> same object
    {
        return false;
    }
    if (newPa->compareVal < thisPa->compareVal) {
        // other has higher possible utility
        return true;
    } else if (newPa->compareVal > thisPa->compareVal) {
        // this has higher possible utility
        return false;
    }
    // Now we are sure that both partial assignments have the same utility
    else if (newPa->plan->getId() > thisPa->plan->getId()) {
        return false;
    } else if (newPa->plan->getId() < thisPa->plan->getId()) {
        return true;
    }
    // Now we are sure that both partial assignments have the same utility and the same plan id
    if (thisPa->unassignedRobotIds.size() < newPa->unassignedRobotIds.size()) {
        return true;
    } else if (thisPa->unassignedRobotIds.size() > newPa->unassignedRobotIds.size()) {
        return false;
    }
    if (newPa->min < thisPa->min) {
        // other has higher actual utility
        return true;
    } else if (newPa->min > thisPa->min) {
        // this has higher actual utility
        return false;
    }

    for (int i = 0; i < thisPa->epRobotsMapping->getSize(); ++i) {
        if (thisPa->epRobotsMapping->getRobots(i)->size() < newPa->epRobotsMapping->getRobots(i)->size()) {
            return true;
        } else if (thisPa->epRobotsMapping->getRobots(i)->size() < newPa->epRobotsMapping->getRobots(i)->size()) {
            return false;
        }
    }
    for (int i = 0; thisPa->epRobotsMapping->getSize(); ++i) {
        for (int j = 0; j < thisPa->epRobotsMapping->getRobots(i)->size(); ++j) {
            if (*(thisPa->epRobotsMapping->getRobots(i)->at(j)) > *(newPa->epRobotsMapping->getRobots(i)->at(j))) {
                return true;
            } else if (*(thisPa->epRobotsMapping->getRobots(i)->at(j)) >
                       *(newPa->epRobotsMapping->getRobots(i)->at(j))) {
                return false;
            }
        }
    }
    return false;
}

string PartialAssignment::toString() {
    stringstream ss;

    ss << "Plan: " << this->plan->getName() << endl;
    ss << "Utility: " << this->min << ".." << this->max << endl;
    ss << "unassignedRobots: ";
    for (auto& robot : this->unassignedRobotIds) {
        ss << robot << " ";
    }
    ss << endl;
    // shared_ptr<vector<EntryPoint*> > ownEps = this->epRobotsMapping->getEntryPoints();
    vector<const supplementary::AgentID*> robots;

    for (int i = 0; i < this->epRobotsMapping->getSize(); ++i) {
        robots = (*this->epRobotsMapping->getRobots(i));
        ss << "EPid: " << this->epRobotsMapping->getEp(i)->getId()
           << " Task: " << this->epRobotsMapping->getEp(i)->getTask()->getName()
           << " minCar: " << this->dynCardinalities[i]->getMin() << " maxCar: "
           << (this->dynCardinalities[i]->getMax() == INFINIT ? "*" : to_string(this->dynCardinalities[i]->getMax()))
           << " Assigned Robots: ";
        for (auto& robot : robots) {
            ss << robot << " ";
        }
        ss << endl;
    }

    ss << this->epRobotsMapping->toString();
    ss << "HashCode: " << this->getHash() << endl;
    return ss.str();
}

string PartialAssignment::assignmentCollectionToString() {
    return "PA: \n" + toString();
}

/**
 * little helper to calculate the y-th power of x with integers
 */
int PartialAssignment::pow(int x, int y) {
    int ret = 1;
    for (int i = 0; i < y; i++) {
        ret *= x;
    }
    return ret;
}

void PartialAssignment::setMax(double max) {
    this->max = max;
    this->compareVal = (long) round(max * PRECISION);
}

} /* namespace alica */
