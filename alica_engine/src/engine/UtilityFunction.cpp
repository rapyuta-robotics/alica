#include "engine/UtilityFunction.h"
#include "engine/Assignment.h"
#include "engine/DefaultUtilityFunction.h"
#include "engine/IRoleAssignment.h"
#include "engine/RunningPlan.h"
#include "engine/TaskRoleStruct.h"
#include "engine/USummand.h"
#include "engine/UtilityInterval.h"
#include "engine/collections/AssignmentCollection.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Plan.h"
#include "engine/model/Role.h"
#include "engine/model/RoleSet.h"
#include "engine/model/RoleTaskMapping.h"
#include "engine/model/Task.h"
#include "engine/teammanager/TeamManager.h"

namespace alica
{

using std::pair;

UtilityFunction::UtilityFunction(const std::string& name, std::list<USummand*> utilSummands, double priorityWeight, double similarityWeight, const Plan* plan)
        : priResult(0.0, 0.0)
        , simUI(0.0, 0.0)
{
    this->ra = nullptr;
    this->ae = nullptr;
    this->name = name;
    this->utilSummands = utilSummands;
    this->priorityWeight = priorityWeight;
    this->similarityWeight = similarityWeight;
    this->plan = plan;
}

UtilityFunction::~UtilityFunction()
{
    for (auto summand : utilSummands) {
        delete summand;
    }
}

std::list<USummand*>& UtilityFunction::getUtilSummands()
{
    return utilSummands;
}

void UtilityFunction::setUtilSummands(list<USummand*> utilSummands)
{
    this->utilSummands = utilSummands;
}

/**
 * Evaluates the utility function according to the priorities of the assigned
 * roles and according to the similarity, if an oldRP is given and according to all
 * other utility summands of this utility function.
 */
double UtilityFunction::eval(const RunningPlan* newRp, const RunningPlan* oldRp)
{
    // Invalid Assignments have an Utility of -1 changed from 0 according to specs
    if (!newRp->getAssignment()->isValid()) {
        return -1.0;
    }
    UtilityInterval sumOfUI(0.0, 0.0);
    double sumOfWeights = 0.0;

    // Sum up priority summand
    UtilityInterval prioUI = this->getPriorityResult(&*newRp->getAssignment());
    if (prioUI.getMax() < 0.0) {
        // one robot has a negative priority for his task -> -1.0 for the complete assignment
        return -1;
    }
    sumOfUI.setMax(sumOfUI.getMax() + this->priorityWeight * prioUI.getMax());
    sumOfUI.setMin(sumOfUI.getMin() + this->priorityWeight * prioUI.getMin());
    sumOfWeights += this->priorityWeight;

    // Sum up all normal utility summands
    UtilityInterval curUI;
    for (int i = 0; i < static_cast<int>(this->utilSummands.size()); ++i) {
        auto iter = utilSummands.begin();
        advance(iter, i);
        curUI = (*iter)->eval(&newRp.getAssignment());
        // if a summand deny assignment, return -1 for forbidden assignments
        if (curUI.getMax() == -1.0) {
            return -1.0;
        }
        sumOfWeights += (*iter)->getWeight();
        sumOfUI.setMax(sumOfUI.getMax() + (*iter)->getWeight() * curUI.getMax());
        sumOfUI.setMin(sumOfUI.getMin() + (*iter)->getWeight() * curUI.getMin());
    }

    if (oldRp != nullptr && this->similarityWeight > 0) {
        // Sum up similarity summand
        UtilityInterval simUI = getSimilarity(&newRp.getAssignment(), &oldRp.getAssignment());
        sumOfUI.setMax(sumOfUI.getMax() + this->similarityWeight * simUI.getMax());
        sumOfUI.setMin(sumOfUI.getMin() + this->similarityWeight * simUI.getMin());
        sumOfWeights += this->similarityWeight;
    }

    // Normalize to 0..1
    if (sumOfWeights > 0.0) {
        sumOfUI.setMax(sumOfUI.getMax() / sumOfWeights);
        sumOfUI.setMin(sumOfUI.getMin() / sumOfWeights);
        // Min == Max because RP.Assignment must be an complete Assignment!
        if ((sumOfUI.getMax() - sumOfUI.getMin()) > DIFFERENCETHRESHOLD) {
            std::cerr << "UF: The utility min and max value differs more than " << DIFFERENCETHRESHOLD << " for an Assignment!" << std::endl;
        }
        return sumOfUI.getMax();
    }

    return 0.0;
}

/**
 * Evaluates the utility function according to the priorities of the assigned
 * roles and according to the similarity, if an oldAss is given.
 * ATTENTION PLZ: Return value is only significant with respect to current Utility of oldAss! (SimilarityMeasure)
 * @return The utility interval
 */
UtilityInterval UtilityFunction::eval(IAssignment* newAss, const Assignment* oldAss)
{
    UtilityInterval sumOfUI(0.0, 0.0);
    double sumOfWeights = 0.0;

    // Sum up priority summand
    UtilityInterval prioUI = this->getPriorityResult(newAss);
    if (prioUI.getMax() == -1.0) {
        // one robot have a negativ priority for his task -> (-1.0, -1.0) for the complete assignment
        return prioUI;
    }
    sumOfUI.setMax(sumOfUI.getMax() + this->priorityWeight * prioUI.getMax());
    sumOfUI.setMin(sumOfUI.getMin() + this->priorityWeight * prioUI.getMin());
    sumOfWeights += this->priorityWeight;
    // Sum up all normal utility summands
    UtilityInterval curUI;
    for (int i = 0; i < static_cast<int>(this->utilSummands.size()); ++i) {
        auto iter = utilSummands.begin();
        advance(iter, i);
        curUI = (*iter)->eval(newAss);
        // if a summand deny assignment, return -1 for forbidden assignments
        if (curUI.getMax() == -1.0) {
            sumOfUI.setMax(-1.0);
            return sumOfUI;
        }
        sumOfWeights += (*iter)->getWeight();
        sumOfUI.setMax(sumOfUI.getMax() + (*iter)->getWeight() * curUI.getMax());
        sumOfUI.setMin(sumOfUI.getMin() + (*iter)->getWeight() * curUI.getMin());
    }
    if (oldAss != nullptr && this->similarityWeight > 0) {
        // Sum up similarity summand
        UtilityInterval simUI = getSimilarity(newAss, oldAss);
        sumOfUI.setMax(sumOfUI.getMax() + this->similarityWeight * simUI.getMax());
        sumOfUI.setMin(sumOfUI.getMin() + this->similarityWeight * simUI.getMin());
        sumOfWeights += this->similarityWeight;
    }
    if (sumOfWeights > 0.0) {
        sumOfUI.setMax(sumOfUI.getMax() / sumOfWeights);
        sumOfUI.setMin(sumOfUI.getMin() / sumOfWeights);

        return sumOfUI;
    }

    sumOfUI.setMin(0.0);
    sumOfUI.setMax(0.0);
    return sumOfUI;
}

/**
 * Updates the utility function according to the priorities of the assigned
 * roles and according to the similarity, if an oldAss is given.
 * @return void
 */
void UtilityFunction::updateAssignment(IAssignment* newAss, IAssignment* oldAss)
{
    UtilityInterval utilityInterval = this->eval(newAss, oldAss);
    newAss->setMin(utilityInterval.getMin());
    newAss->setMax(utilityInterval.getMax());
}

void UtilityFunction::cacheEvalData()
{
    if (this->utilSummands.size() != 0) // == null for default utility function
    {
        for (int i = 0; i < static_cast<int>(this->utilSummands.size()); ++i) {
            auto iter = this->utilSummands.begin();
            advance(iter, i);
            (*iter)->cacheEvalData();
        }
    }
}

/**
 * Initializes the '(Task x Role) -> Priority'-Dictionary and the
 * 'Role -> Highest Priority'-Dictionary for each role of the current roleset.
 * @return void
 */
void UtilityFunction::init(AlicaEngine* ae)
{
    // CREATE MATRIX && HIGHEST PRIORITY ARRAY
    // init dicts
    this->roleHighestPriorityMap.clear();
    this->priorityMatrix.clear();
    const RoleSet* roleSet = ae->getRoleSet();
    int64_t taskId;
    int64_t roleId;
    double curPrio;

    for (const RoleTaskMapping* rtm : roleSet->getRoleTaskMappings()) {
        roleId = rtm->getRole()->getId();
        this->roleHighestPriorityMap.insert(std::pair<int64_t, double>(roleId, 0.0));
        for (const EntryPoint* ep : plan->getEntryPoints()) {
            taskId = ep->getTask()->getId();
            auto iter = rtm->getTaskPriorities().find(taskId);
            if (iter == rtm->getTaskPriorities().end()) {
                std::stringstream ss;
                ss << "UF: There is no priority for the task " << taskId << " in the roleTaskMapping of the role " << rtm->getRole()->getName() << " with id "
                   << roleId << "!\n We are in the UF for the plan " << this->plan->getName() << "!" << std::endl;
                AlicaEngine::abort(ss.str());
                return;
            } else {
                curPrio = iter->second;
            }
            priorityMatrix[TaskRoleStruct(taskId, roleId)] = curPrio;
            if (this->roleHighestPriorityMap.at(roleId) < curPrio) {
                this->roleHighestPriorityMap.at(roleId) = curPrio;
            }
        }
        // Add Priority for Idle-EntryPoint
        this->priorityMatrix.insert(std::pair<TaskRoleStruct, double>(TaskRoleStruct(Task::IDLEID, roleId), 0.0));
    }
    // c# != null
    // INIT UTILITYSUMMANDS
    if (this->utilSummands.size() != 0) // it is null for default utility function
    {
        for (USummand* utilSum : this->utilSummands) {
            utilSum->init(ae);
        }
    }
    this->ae = ae;
    this->ra = this->ae->getRoleAssignment();
}

/**
 *  Calls Init() for every utiltiyfunction.
 * Is called and the end of AlicaEngine.Init(..), because it
 * needs the current roleset (circular dependency otherwise).
 */
void UtilityFunction::initDataStructures(AlicaEngine* ae)
{
    for (const Plan* p : ae->getPlanRepository()->getPlans()) {
        p->getUtilityFunction()->init(ae);
    }
}

std::string UtilityFunction::toString() const
{
    std::stringstream ss;
    ss << this->name << std::endl;
    ss << "prioW: " << this->priorityWeight << " simW: " << this->similarityWeight << std::endl;
    for (const USummand* utilSummand : this->utilSummands) {
        ss << utilSummand->toString();
    }
    return ss.str();
}

/**
 * Calculates the priority result for the specified Assignment
 * @return the priority result
 */
UtilityInterval UtilityFunction::getPriorityResult(IAssignment* ass)
{
    this->priResult.setMax(0.0);
    this->priResult.setMin(0.0);
    if (this->priorityWeight == 0) {
        return this->priResult;
    }
    // c# != null
    // SUM UP HEURISTIC PART OF PRIORITY UTILITY

    if (ass->getUnassignedRobotIds().size() != 0) // == null, when it is a normal assignment
    {
        for (auto robotID : ass->getUnassignedRobotIds()) {
            this->priResult.setMax(this->priResult.getMax() + this->roleHighestPriorityMap.at(this->ra->getRole(robotID)->getId()));
        }
    }
    // SUM UP DEFINED PART OF PRIORITY UTILITY

    // for better comparability of different utility functions
    int denum = std::min(this->plan->getMaxCardinality(), this->ae->getTeamManager()->getTeamSize());
    int64_t taskId;
    int64_t roleId;
    //	shared_ptr<vector<EntryPoint*> > eps = ass->getEntryPoints();
    double curPrio = 0;
    for (short i = 0; i < ass->getEntryPointCount(); ++i) {
        const EntryPoint* ep = ass->getEpRobotsMapping()->getEp(i);
        taskId = ep->getTask()->getId();
        auto robotList = ass->getUniqueRobotsWorkingAndFinished(ep);
        for (auto robot : *robotList) {
            roleId = this->ra->getRole(robot)->getId();
            TaskRoleStruct lookup(taskId, roleId);
            auto mit = priorityMatrix.find(lookup);
            if (mit != priorityMatrix.end()) {
                curPrio = mit->second;
            }
            if (curPrio < 0.0) // because one Robot has a negative priority for his task
            {
                this->priResult.setMin(-1.0);
                this->priResult.setMax(-1.0);
                return this->priResult;
            }
            this->priResult.setMin(this->priResult.getMin() + curPrio);
#ifdef UFDEBUG
            std::cout << "UF: taskId:" << taskId << " roleId:" << roleId << " prio: " << curPrio << std::endl;
#endif
        }
    }
#ifdef UFDEBUG
    std::cout << "##" << std::endl;
    std::cout << "UF: prioUI.Min = " << priResult.getMin() << std::endl;
    std::cout << "UF: prioUI.Max = " << priResult.getMax() << std::endl;
    std::cout << "UF: denum = " << denum << std::endl;
#endif
    priResult.setMax(priResult.getMax() + priResult.getMin());
    if (denum != 0) {
        priResult.setMin(priResult.getMin() / denum);
        priResult.setMax(priResult.getMax() / denum);
    }
#ifdef UFDEBUG
    std::cout << "UF: prioUI.Min = " << priResult.getMin() << std::endl;
    std::cout << "UF: prioUI.Max = " << priResult.getMax() << std::endl;
    std::cout << "##" << std::endl;
#endif
    return priResult;
}

pair<vector<double>, double>* UtilityFunction::differentiate(IAssignment* newAss)
{
    return nullptr;
}

/**
 * Evaluates the similarity of the new Assignment to the old Assignment
 * @return The result of the evaluation
 */
UtilityInterval UtilityFunction::getSimilarity(const IAssignment* newAss, const Assignment* oldAss)
{
    simUI.setMax(0.0);
    simUI.setMin(0.0);
    // Calculate the similarity to the old Assignment
    int numOldAssignedRobots = 0;
    // shared_ptr<vector<EntryPoint*> > oldAssEps = oldAss->getEntryPoints();
    for (short i = 0; i < oldAss->getEntryPointCount(); ++i) {
        const EntryPoint* ep = oldAss->getEntryPoint(i);
        // for normalisation
        AgentGrp oldRobots;
        AgentGrp newRobots;
        oldAss->getAgentsWorkingAndFinished(ep, oldRobots);

        numOldAssignedRobots += oldRobots->size();

        newAss->getAgentsWorkingAndFinished(ep, newRobots);

        if (!newRobots.empty()) {
            for (AgentIdConstPtr oldRobot : oldRobots) {
                if (find_if(newRobots.begin(), newRobots.end(), [oldRobot](AgentIdConstPtr id) { return *oldRobot == *id; }) != newRobots->end()) {
                    simUI.setMin(simUI.getMin() + 1);
                } else if (ep->getMaxCardinality() > static_cast<int>(newRobots.size()) &&
                           find_if(newAss->getUnassignedRobotIds().begin(), newAss->getUnassignedRobotIds().end(),
                                   [oldRobot](AgentIdConstPtr id) { return *oldRobot == *id; }) != newAss->getUnassignedRobotIds().end()) {
                    simUI.setMax(simUI.getMax() + 1);
                }
            }
        }
    }

    simUI.setMax(simUI.getMax() + simUI.getMin());
    // Normalise if possible
    if (numOldAssignedRobots > 0) {
        simUI.setMin(simUI.getMin() / numOldAssignedRobots);
        simUI.setMax(simUI.getMax() / numOldAssignedRobots);
    }

    return simUI;
}

} /* namespace alica */
