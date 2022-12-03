#include "engine/UtilityFunction.h"

#include "engine/Assignment.h"
#include "engine/DefaultUtilityFunction.h"
#include "engine/IAlicaWorldModel.h"
#include "engine/IRoleAssignment.h"
#include "engine/RunningPlan.h"
#include "engine/TaskRoleStruct.h"
#include "engine/USummand.h"
#include "engine/UtilityInterval.h"
#include "engine/logging/Logging.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Plan.h"
#include "engine/model/Role.h"
#include "engine/model/RoleSet.h"
#include "engine/model/Task.h"
#include "engine/planselector/IAssignment.h"

namespace alica
{

using std::pair;

UtilityFunction::UtilityFunction(double priorityWeight, double similarityWeight, const Plan* plan)
        : _plan(plan)
        , _roleSet(nullptr)
        , _roleAssignment(nullptr)
        , _teamManager(nullptr)
        , _priorityWeight(priorityWeight)
        , _similarityWeight(similarityWeight)
{
}

UtilityFunction::~UtilityFunction() {}

/**
 * Evaluates the utility function according to the priorities of the assigned
 * roles and according to the similarity, if an oldAss is given.
 * ATTENTION PLZ: Return value is only significant with respect to current Utility of oldAss! (SimilarityMeasure)
 * @return The utility interval
 */
UtilityInterval UtilityFunction::eval(const PartialAssignment* newAss, const Assignment* oldAss, const Blackboard* worldModels) const
{
    if (!newAss->isValid()) {
        return UtilityInterval(-1.0, -1.0);
    }
    UtilityInterval sumOfUI(0.0, 0.0);
    double sumOfWeights = 0.0;

    IAssignment wrapper{newAss};
    // Sum up priority summand
    UtilityInterval prioUI = getPriorityResult(wrapper);
    if (prioUI.getMax() <= -1.0) {
        // one robot have a negativ priority for his task -> (-1.0, -1.0) for the complete assignment
        return prioUI;
    }
    sumOfUI += _priorityWeight * prioUI;

    sumOfWeights += _priorityWeight;
    // Sum up all normal utility summands
    UtilityInterval curUI;

    for (const std::unique_ptr<USummand>& us : _utilSummands) {
        curUI = us->eval(wrapper, oldAss, worldModels);
        // if a summand deny assignment, return -1 for forbidden assignments
        if (curUI.getMax() <= -1.0) {
            sumOfUI.setMax(-1.0);
            return sumOfUI;
        }
        sumOfWeights += us->getWeight();
        sumOfUI += us->getWeight() * curUI;
    }
    if (oldAss != nullptr && _similarityWeight > 0) {
        // Sum up similarity summand
        UtilityInterval simUI = getSimilarity(wrapper, oldAss);
        sumOfUI += _similarityWeight * simUI;
        sumOfWeights += _similarityWeight;
    }
    if (sumOfWeights > 0.0) {
        sumOfUI /= sumOfWeights;
        return sumOfUI;
    }
    return UtilityInterval(0.0, 0.0);
}

void UtilityFunction::cacheEvalData(const Blackboard* worldModels)
{
    for (const std::unique_ptr<USummand>& us : _utilSummands) {
        us->cacheEvalData(worldModels);
    }
}

/**
 * Initializes the '(Task x Role) -> Priority'-Dictionary and the
 * 'Role -> Highest Priority'-Dictionary for each role of the current roleset.
 * @return void
 */
void UtilityFunction::init(const RoleSet* roleSet, const IRoleAssignment& roleAssignment, const TeamManager& teamManager)
{
    _roleSet = roleSet;
    _roleAssignment = &roleAssignment;
    _teamManager = &teamManager;

    // CREATE MATRIX && HIGHEST PRIORITY ARRAY
    // init dicts
    _roleHighestPriorityMap.clear();
    _priorityMatrix.clear();

    for (const Role* role : _roleSet->getRoles()) {
        const int64_t roleId = role->getId();
        int64_t taskId;
        _roleHighestPriorityMap.insert(std::pair<int64_t, double>(roleId, 0.0));
        for (const EntryPoint* ep : _plan->getEntryPoints()) {
            taskId = ep->getTask()->getId();
            double curPrio = role->getPriority(taskId);
            _priorityMatrix[TaskRoleStruct(taskId, roleId)] = curPrio;
            if (_roleHighestPriorityMap.at(roleId) < curPrio) {
                _roleHighestPriorityMap.at(roleId) = curPrio;
            }
        }
        // Add Priority for Idle-EntryPoint
        _priorityMatrix.insert(std::pair<TaskRoleStruct, double>(TaskRoleStruct(Task::IDLEID, roleId), 0.0));
    }
}

/**
 *  Calls Init() for every utility function.
 * Is called and the end of AlicaEngine.Init(..), because it
 * needs the current roleset (circular dependency otherwise).
 */
void UtilityFunction::initDataStructures(
        const PlanRepository& planRepository, const RoleSet* roleSet, const IRoleAssignment& roleAssignment, const TeamManager& teamManager)
{
    for (const Plan* p : planRepository.getPlans()) {
        p->getUtilityFunction()->init(roleSet, roleAssignment, teamManager);
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
    for (AgentId agentID : ass.getUnassignedAgents()) {
        const auto highestPriority = _roleHighestPriorityMap.find(_roleAssignment->getRole(agentID)->getId());
        assert(highestPriority != _roleHighestPriorityMap.end());
        priResult.setMax(priResult.getMax() + highestPriority->second);
    }
    // SUM UP DEFINED PART OF PRIORITY UTILITY

    for (int i = 0; i < ass.getEntryPointCount(); ++i) {
        const EntryPoint* ep = ass.getEntryPoint(i);
        const int64_t taskId = ep->getTask()->getId();

        for (AgentId agent : ass.getUniqueAgentsWorkingAndFinished(ep)) {
            double curPrio = 0;
            const int64_t roleId = _roleAssignment->getRole(agent)->getId();
            const auto mit = _priorityMatrix.find(TaskRoleStruct{taskId, roleId});
            if (mit != _priorityMatrix.end()) {
                curPrio = mit->second;
            }
            if (curPrio < 0.0) // because one Robot has a negative priority for his task
            {
                return UtilityInterval(-1.0, -1.0);
            }
            priResult.setMin(priResult.getMin() + curPrio);

            Logging::logDebug("UF") << "taskId:" << taskId << " roleId:" << roleId << " prio: " << curPrio;
        }
    }
    // for better comparability of different utility functions
    int denum = std::min(_plan->getMaxCardinality(), _teamManager->getTeamSize());

    Logging::logDebug("UF") << "##\n"
                            << "prioUI = " << priResult;
    Logging::logDebug("UF") << "denum = " << denum;

    priResult.setMax(priResult.getMax() + priResult.getMin());
    if (denum != 0) {
        priResult /= denum;
    }

    Logging::logDebug("UF") << "prioUI = " << priResult << "\n"
                            << "##";
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
        PartialAssignmentSuccessView newRobots = newAss.getAgentsWorkingAndFinished(ep);
        // for normalisation
        numOldAssignedRobots += oldRobots.size();

        if (!newRobots.empty()) {
            for (AgentId oldRobot : oldRobots) {
                if (std::find(newRobots.begin(), newRobots.end(), oldRobot) != newRobots.end()) {
                    simUI.setMin(simUI.getMin() + 1);
                } else if (ep->getMaxCardinality() > newRobots.size() && std::find(newAss.getUnassignedAgents().begin(), newAss.getUnassignedAgents().end(),
                                                                                 oldRobot) != newAss.getUnassignedAgents().end()) {
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
    ss << "UtilityFunction: prioW: " << uf._priorityWeight << " simW: " << uf._similarityWeight << std::endl;
    for (const std::unique_ptr<USummand>& utilSummand : uf._utilSummands) {
        ss << utilSummand->toString();
    }
    return ss;
}

} /* namespace alica */
