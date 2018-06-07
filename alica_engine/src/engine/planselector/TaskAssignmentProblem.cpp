#include "engine/planselector/TaskAssignmentProblem.h"

#include "engine/AlicaEngine.h"
#include "engine/Assignment.h"
#include "engine/TeamObserver.h"
#include "engine/UtilityFunction.h"
#include "engine/planselector/PartialAssignment.h"
#include "engine/planselector/PartialAssignmentPool.h"
#include <engine/collections/SuccessCollection.h>
#include <engine/model/EntryPoint.h>
#include <engine/model/Plan.h>
#include <engine/model/Task.h>
#include <engine/teammanager/TeamManager.h>

#include <supplementary/AgentID.h>

#include <alica_common_config/debug_output.h>

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
 * @param planList Plans to build an assignment for
 * @param paraAgents agents to build an assignment for
 * @param a bool
 */
TaskAssignmentProblem::TaskAssignmentProblem(const AlicaEngine* engine, const PlanGrp& planList, const AgentGrp& paraAgents, PartialAssignmentPool& pool)
        : _agents(paraAgents)
        , _plans(planList)
#ifdef EXPANSIONEVAL
        , _expansionCount(0)
#endif
        , _to(engine->getTeamObserver())
        , _tm(engine->getTeamManager())
        , _pool(pool)
{
    // sort agent ids ascending
    std::sort(_agents.begin(), _agents.end(), supplementary::AgentIDComparator());

    _successData.reserve(_plans.size());
    _fringe.reserve(_plans.size());

    for (const Plan* curPlan : _plans) {
        // prep successinfo for this plan
        _successData.push_back(_to->createSuccessCollection(curPlan));
        // allow caching of eval data
        curPlan->getUtilityFunction()->cacheEvalData();
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
    auto simplePlanTreeMap = _to->getTeamPlanTrees();
    // this call should only be made before the search starts
    assert(_fringe.size() == _plans.size());
    // ASSIGN PREASSIGNED OTHER ROBOTS
    int i = 0;
    bool changed = false;
    for (PartialAssignment* curPa : _fringe) {
        if (addAlreadyAssignedRobots(curPa, &(*simplePlanTreeMap))) {
            // revaluate this pa
            curPa->evaluate(nullptr);
            changed = true;
        }
        ++i;
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
    ALICA_DEBUG_MSG("TA: Calculating next best PartialAssignment...");
    PartialAssignment* calculatedPa = calcNextBestPartialAssignment(oldAss);

    if (calculatedPa == nullptr) {
        return Assignment();
    }

    ALICA_DEBUG_MSG("TA: ... calculated this PartialAssignment:\n" << *calculatedPa);

    Assignment newAss = Assignment(*calculatedPa);

    ALICA_DEBUG_MSG("TA: Return this Assignment to PS:" << newAss);

    return newAss;
}

std::string TaskAssignmentProblem::toString() const
{
    std::stringstream ss;
    ss << std::endl;
    ss << "--------------------TA:--------------------" << std::endl;
    ss << "NumRobots: " << _agents.size() << std::endl;
    ss << "RobotIDs: ";
    for (AgentIDConstPtr id : _agents) {
        ss << *id << " ";
    }
    ss << std::endl;
    ss << "Initial Fringe (Count " << _fringe.size() << "):" << std::endl;
    ss << "{";
    for (PartialAssignment* pa : _fringe) // Initial PartialAssignments
    {
        ss << *pa << std::endl;
    }
    ss << "}" << std::endl;
    ss << "-------------------------------------------" << std::endl;
    ;
    return ss.str();
}

PartialAssignment* TaskAssignmentProblem::calcNextBestPartialAssignment(const Assignment* oldAss)
{
    PartialAssignment* goal = nullptr;
    while (!_fringe.empty() && goal == nullptr) {
        PartialAssignment* curPa = _fringe.back();
        _fringe.pop_back();
        ALICA_DEBUG_MSG("<--- TA: NEXT PA from fringe:");
        ALICA_DEBUG_MSG(*curPa << "--->");

        if (curPa->isGoal()) {
            goal = curPa;
        } else {
            ALICA_DEBUG_MSG("<--- TA: BEFORE fringe exp:");
            ALICA_DEBUG_MSG(_fringe << "--->");
            curPa->expand(_fringe, _pool, oldAss);
            ALICA_DEBUG_MSG("<--- TA: AFTER fringe exp:" << std::endl << "TA: fringe size " << _fringe.size());
            ALICA_DEBUG_MSG(_fringe << "--->");
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
bool TaskAssignmentProblem::addAlreadyAssignedRobots(
        PartialAssignment* pa, std::map<const supplementary::AgentID*, std::shared_ptr<SimplePlanTree>, supplementary::AgentIDComparator>* simplePlanTreeMap)
{
    AgentIDConstPtr ownAgentId = _tm->getLocalAgentID();
    bool haveToRevalute = false;
    int i = 0;
    for (const supplementary::AgentID* agent : _agents) {
        if (*ownAgentId == *agent) {
            continue;
        }
        auto iter = simplePlanTreeMap->find(agent);
        if (iter != simplePlanTreeMap->end()) {
            if (pa->addIfAlreadyAssigned(iter->second.get(), agent, i)) {
                haveToRevalute = true;
            }
        }
        ++i;
    }
    return haveToRevalute;
}

} /* namespace alica */
