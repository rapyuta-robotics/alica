#include "engine/planselector/TaskAssignmentProblem.h"

#include "engine/Assignment.h"
#include "engine/TeamObserver.h"
#include "engine/UtilityFunction.h"
#include "engine/logging/Logging.h"
#include "engine/planselector/PartialAssignment.h"
#include "engine/planselector/PartialAssignmentPool.h"
#include <engine/collections/SuccessCollection.h>
#include <engine/model/EntryPoint.h>
#include <engine/model/Plan.h>
#include <engine/model/Task.h>
#include <engine/teammanager/TeamManager.h>

namespace alica
{
std::ostream& operator<<(std::ostream& out, const std::vector<PartialAssignment*>& pas)
{
    for (const PartialAssignment* pa : pas) {
        out << *pa << " ";
    }

    return out;
}

TaskAssignmentProblem::~TaskAssignmentProblem() {}

/**
 * Constructor of a new TaskAssignment
 *
 * @param planList Plans to build an assignment for
 * @param paraAgents agents to build an assignment for
 * @param a bool
 */
TaskAssignmentProblem::TaskAssignmentProblem(const TeamObserver& teamObserver, const TeamManager& teamManager, const PlanGrp& planList,
        const AgentGrp& paraAgents, PartialAssignmentPool& pool, const Blackboard* globalBlackboard)
        : _agents(paraAgents)
        , _plans(planList)
        , _globalBlackboard(globalBlackboard)
#ifdef EXPANSIONEVAL
        , _expansionCount(0)
#endif
        , _teamObserver(teamObserver)
        , _teamManager(teamManager)
        , _pool(pool)
{
    // sort agent ids ascending
    std::sort(_agents.begin(), _agents.end());

    _successData.reserve(_plans.size());
    _fringe.reserve(_plans.size());

    for (const Plan* curPlan : _plans) {
        // prep successinfo for this plan
        _successData.push_back(_teamObserver.createSuccessCollection(curPlan));
        // allow caching of eval data
        curPlan->getUtilityFunction()->cacheEvalData(_globalBlackboard);
        // seed the fringe with a partial assignment
        PartialAssignment* curPa = _pool.getNext();

        curPa->prepare(curPlan, this);

        _fringe.push_back(curPa);
    }
    stable_sort(_fringe.begin(), _fringe.end(), PartialAssignment::compare);
}

void TaskAssignmentProblem::preassignOtherAgents()
{
    // TODO: fix this call
    const auto& simplePlanTreeMap = _teamObserver.getTeamPlanTrees();
    // this call should only be made before the search starts
    assert(_fringe.size() == _plans.size());
    // ASSIGN PREASSIGNED OTHER ROBOTS
    bool changed = false;
    for (PartialAssignment* curPa : _fringe) {
        if (addAlreadyAssignedRobots(curPa, simplePlanTreeMap)) {
            // reevaluate this pa
            curPa->evaluate(nullptr, _globalBlackboard);
            changed = true;
        }
    }
    if (changed) {
        stable_sort(_fringe.begin(), _fringe.end(), PartialAssignment::compare);
    }
}

/**
 * Gets the Assignment with next best utility
 * @param oldAss old Assignment
 * @return An Assignment for the plan
 */
Assignment TaskAssignmentProblem::getNextBestAssignment(const Assignment* oldAss)
{
    PartialAssignment* calculatedPa = calcNextBestPartialAssignment(oldAss);

    if (calculatedPa == nullptr) {
        return Assignment();
    }

    Assignment newAss = Assignment(*calculatedPa);

    return newAss;
}

PartialAssignment* TaskAssignmentProblem::calcNextBestPartialAssignment(const Assignment* oldAss)
{
    PartialAssignment* goal = nullptr;
    while (!_fringe.empty() && goal == nullptr) {
        PartialAssignment* curPa = _fringe.back();
        _fringe.pop_back();

        if (curPa->isGoal()) {
            goal = curPa;
        } else {
            curPa->expand(_fringe, _pool, oldAss, _globalBlackboard);
        }
#ifdef EXPANSIONEVAL
        ++_expansionCount;
#endif
    }
    return goal;
}
/**
 * If any robot has already assigned itself, this method updates the partial assignment accordingly.
 * @param pa A PartialAssignment
 * @param simplePlanTreeMap never try to delete this
 * @return True if any robot has already assigned itself, false otherwise
 */
bool TaskAssignmentProblem::addAlreadyAssignedRobots(PartialAssignment* pa, const std::map<AgentId, std::unique_ptr<SimplePlanTree>>& simplePlanTreeMap)
{
    AgentId ownAgentId = _teamManager.getLocalAgentID();
    bool haveToRevalute = false;

    for (int i = 0; i < static_cast<int>(_agents.size()); ++i) {
        if (ownAgentId == _agents[i]) {
            continue;
        }
        auto iter = simplePlanTreeMap.find(_agents[i]);
        if (iter != simplePlanTreeMap.end()) {
            if (pa->addIfAlreadyAssigned(iter->second.get(), _agents[i], i)) {
                haveToRevalute = true;
            }
        }
    }
    return haveToRevalute;
}

std::ostream& operator<<(std::ostream& out, const TaskAssignmentProblem& tap)
{
    out << std::endl;
    out << "--------------------TA:--------------------" << std::endl;
    out << "Agent count: " << tap._agents.size() << std::endl;
    out << "AgentIDs: ";
    for (AgentId id : tap._agents) {
        out << id << " ";
    }
    out << std::endl;
    out << "Initial Fringe (Count " << tap._fringe.size() << "):" << std::endl;
    out << "{";
    for (const PartialAssignment* pa : tap._fringe) // Initial PartialAssignments
    {
        out << *pa << std::endl;
    }
    out << "}" << std::endl;
    out << "-------------------------------------------" << std::endl;
    return out;
}

} /* namespace alica */
