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

#include <alica_common_config/debug_output.h>

namespace alica
{

using std::pair;

UtilityFunction::UtilityFunction(double priorityWeight, double similarityWeight, const Plan* plan)
        : _plan(plan)
        , _ra(nullptr)
        , _ae(nullptr)
        , _priorityWeight(priorityWeight)
        , _similarityWeight(similarityWeight)
{
}

UtilityFunction::~UtilityFunction()
{
    for (USummand* summand : utilSummands) {
        delete summand;
    }
}

/**
 * Evaluates the utility function according to the priorities of the assigned
 * roles and according to the similarity, if an oldRP is given and according to all
 * other utility summands of this utility function.
 */
/*double UtilityFunction::eval(const RunningPlan* newRp, const RunningPlan* oldRp) const
{
    // Invalid Assignments have an Utility of -1 changed from 0 according to specs
    if (!newRp->getAssignment()->isValid()) {
        return -1.0;
    }
    return eval(&newRp->getAssignment(), &oldRp->getAssignment()).getMax();
}*/

/**
 * Evaluates the utility function according to the priorities of the assigned
 * roles and according to the similarity, if an oldAss is given.
 * ATTENTION PLZ: Return value is only significant with respect to current Utility of oldAss! (SimilarityMeasure)
 * @return The utility interval
 */
UtilityInterval UtilityFunction::eval(const PartialAssignment* newAss, const Assignment* oldAss) const
{
    UtilityInterval sumOfUI(0.0, 0.0);
    double sumOfWeights = 0.0;

    // Sum up priority summand
    UtilityInterval prioUI = getPriorityResult(newAss);
    if (prioUI.getMax() <= -1.0) {
        // one robot have a negativ priority for his task -> (-1.0, -1.0) for the complete assignment
        return prioUI;
    }
    sumOfUI += _priorityWeight * prioUI;

    sumOfWeights += _priorityWeight;
    // Sum up all normal utility summands
    UtilityInterval curUI;
    IAssignment wrapper{newAss};
    for (USummand* us : _utilSummands) {

        curUI = us->eval(wrapper);
        // if a summand deny assignment, return -1 for forbidden assignments
        if (curUI.getMax() <= -1.0) {
            sumOfUI.setMax(-1.0);
            return sumOfUI;
        }
        sumOfWeights += us->getWeight();
        sumOfUi += us->getWeight() * curUI;
    }
    if (oldAss != nullptr && _similarityWeight > 0) {
        // Sum up similarity summand
        UtilityInterval simUI = getSimilarity(newAss, oldAss);
        sumOfUI += _similarityWeight * simUI;
        sumOfWeights += _similarityWeight;
    }
    if (sumOfWeights > 0.0) {
        sumOfUI /= sumOfWeights;
        return sumOfUI;
    }
    return UtilityInterval(0.0, 0.0);
}

void UtilityFunction::cacheEvalData()
{
    for (USummand* us : _utilSummands) {
        us->cacheEvalData();
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
    _roleHighestPriorityMap.clear();
    _priorityMatrix.clear();
    const RoleSet* roleSet = ae->getRoleSet();
    int64_t taskId;
    int64_t roleId;
    double curPrio;

    for (const RoleTaskMapping* rtm : roleSet->getRoleTaskMappings()) {
        roleId = rtm->getRole()->getId();
        _roleHighestPriorityMap.insert(std::pair<int64_t, double>(roleId, 0.0));
        for (const EntryPoint* ep : plan->getEntryPoints()) {
            taskId = ep->getTask()->getId();
            auto iter = rtm->getTaskPriorities().find(taskId);
            if (iter == rtm->getTaskPriorities().end()) {
                ALICA_ERROR_MSG("UF: There is no priority for the task " << taskId << " in the roleTaskMapping of the role " << rtm->getRole()->getName()
                                                                         << " with id " << roleId << "!\n We are in the UF for the plan " << _plan->getName()
                                                                         << "!");
                AlicaEngine::abort("Error in Utility data, cannot continue.");
                return;
            } else {
                curPrio = iter->second;
            }
            priorityMatrix[TaskRoleStruct(taskId, roleId)] = curPrio;
            if (_roleHighestPriorityMap.at(roleId) < curPrio) {
                _roleHighestPriorityMap.at(roleId) = curPrio;
            }
        }
        // Add Priority for Idle-EntryPoint
        _priorityMatrix.insert(std::pair<TaskRoleStruct, double>(TaskRoleStruct(Task::IDLEID, roleId), 0.0));
    }
    _ae = ae;
    _ra = _ae->getRoleAssignment();
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

/**
 * Calculates the priority result for the specified Assignment
 * @return the priority result
 */
UtilityInterval UtilityFunction::getPriorityResult(IAssignment ass) const
{
    UtilityInterval priResult(0.0, 0.0);
    if (_priorityWeight == 0) {
        return priResult;
    }
    // SUM UP HEURISTIC PART OF PRIORITY UTILITY

    for (AgentIdConstPtr robotID : ass.getUnassignedAgents()) {
        priResult.setMax(priResult.getMax() + _roleHighestPriorityMap[_ra->getRole(robotID)->getId())];
    }
    // SUM UP DEFINED PART OF PRIORITY UTILITY

    // for better comparability of different utility functions
    int denum = std::min(_plan->getMaxCardinality(), _ae->getTeamManager()->getTeamSize());

    for (int i = 0; i < ass.getEntryPointCount(); ++i) {
        const EntryPoint* ep = ass.getEntryPoint(i);
        const int64_t taskId = ep->getTask()->getId();

        for (AgentIDConstPtr agent : ass.getUniqueRobotsWorkingAndFinished(ep)) {
            double curPrio = 0;
            const int64_t roleId = _ra->getRole(agent)->getId();
            TaskRoleStruct lookup(taskId, roleId);
            auto mit = priorityMatrix.find(lookup);
            if (mit != priorityMatrix.end()) {
                curPrio = mit->second;
            }
            if (curPrio < 0.0) // because one Robot has a negative priority for his task
            {
                return UtilityInterval(-1.0, -1.0);
            }
            priResult.setMin(priResult.getMin() + curPrio);

            ALICA_DEBUG_MSG("UF: taskId:" << taskId << " roleId:" << roleId << " prio: " << curPrio);
        }
    }

    ALICA_DEBUG_MSG("##" << std::endl << "UF: prioUI = " << priResult);
    ALICA_DEBUG_MSG("UF: denum = " << denum);

    priResult.setMax(priResult.getMax() + priResult.getMin());
    if (denum != 0) {
        priResult /= denum;
    }

    ALICA_DEBUG_MSG("UF: prioUI = " << priResult);
    ALICA_DEBUG_MSG("##");
    return priResult;
}

/**
 * Evaluates the similarity of the new Assignment to the old Assignment
 * @return The result of the evaluation
 */
UtilityInterval UtilityFunction::getSimilarity(IAssignment newAss, const Assignment* oldAss) const
{
    UtilityInterval simUI(0.0, 0.0);
    // Calculate the similarity to the old Assignment
    int numOldAssignedRobots = 0;
    // shared_ptr<vector<EntryPoint*> > oldAssEps = oldAss->getEntryPoints();
    for (short i = 0; i < oldAss->getEntryPointCount(); ++i) {
        const EntryPoint* ep = oldAss->getEntryPoint(i);
        AssignmentSuccessView oldRobots = oldAss->getAgentsWorkingAndFinished(ep);
        PartialAssignmentSuccessView newRobots = newAss->getAgentsWorkingAndFinished(ep);
        // for normalisation
        numOldAssignedRobots += oldRobots.size();

        if (!newRobots.empty()) {
            for (AgentIdConstPtr oldRobot : oldRobots) {
                if (find_if(newRobots.begin(), newRobots.end(), [oldRobot](AgentIdConstPtr id) { return *oldRobot == *id; }) != newRobots->end()) {
                    simUI.setMin(simUI.getMin() + 1);
                } else if (ep->getMaxCardinality() > static_cast<int>(newRobots.size()) &&
                           find_if(newAss->getUnassignedAgents().begin(), newAss->getUnassignedAgents().end(),
                                   [oldRobot](AgentIdConstPtr id) { return *oldRobot == *id; }) != newAss->getUnassignedRobotIds().end()) {
                    simUI.setMax(simUI.getMax() + 1);
                }
            }
        }
    }

    simUI.setMax(simUI.getMax() + simUI.getMin());
    // Normalise if possible
    if (numOldAssignedRobots > 0) {
        simUI /= numOldAssignedRobots;
    }

    return simUI;
}

std::stringstream& operator<<(std::stringstream& ss, const UtilityFunction& uf)
{

    ss << "UtilityFunction: prioW: " << uf.priorityWeight << " simW: " << uf.similarityWeight << std::endl;
    for (const USummand* utilSummand : uf.utilSummands) {
        ss << utilSummand.toString();
    }
    return ss;
}

} /* namespace alica */
