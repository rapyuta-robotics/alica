#include "engine/planselector/TaskAssignmentProblem.h"

#include "engine/AlicaEngine.h"
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
 * @param planList Plans to build an assignment for
 * @param paraAgents agents to build an assignment for
 * @param a bool
 */
TaskAssignmentProblem::TaskAssignmentProblem(
        AlicaEngine* engine, const PlanGrp& planList, const AgentGrp& paraAgents, PartialAssignmentPool& pool, const IAlicaWorldModel* wm)
        : _agents(paraAgents)
        , _plans(planList)
        , _wm(wm)
#ifdef EXPANSIONEVAL
        , _expansionCount(0)
#endif
        , _to(engine->getTeamObserver())
        , _tm(engine->getTeamManager())
        , _pool(pool)
{
    // sort agent ids ascending
    std::sort(_agents.begin(), _agents.end());

    _successData.reserve(_plans.size());
    _fringe.reserve(_plans.size());

    for (const Plan* curPlan : _plans) {
        // prep successinfo for this plan
        _successData.push_back(_to.createSuccessCollection(curPlan));
        // allow caching of eval data
        curPlan->getUtilityFunction()->cacheEvalData(_wm);
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
    const auto& simplePlanTreeMap = _to.getTeamPlanTrees();
    // this call should only be made before the search starts
    assert(_fringe.size() == _plans.size());
    // ASSIGN PREASSIGNED OTHER ROBOTS
    int i = 0;
    bool changed = false;
    for (PartialAssignment* curPa : _fringe) {
        if (addAlreadyAssignedRobots(curPa, simplePlanTreeMap)) {
            // reevaluate this pa
            curPa->evaluate(nullptr, _wm);
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
    Logging::logDebug("TA") << "Calculating next best PartialAssignment...";
    PartialAssignment* calculatedPa = calcNextBestPartialAssignment(oldAss);

    if (calculatedPa == nullptr) {
        return Assignment();
    }

    Logging::logDebug("TA") << "... calculated this PartialAssignment:\n" << *calculatedPa;

    Assignment newAss = Assignment(*calculatedPa);

    Logging::logDebug("TA") << "Return this Assignment to PS:" << newAss;

    return newAss;
}

PartialAssignment* TaskAssignmentProblem::calcNextBestPartialAssignment(const Assignment* oldAss)
{
    PartialAssignment* goal = nullptr;
    while (!_fringe.empty() && goal == nullptr) {
        PartialAssignment* curPa = _fringe.back();
        _fringe.pop_back();
        Logging::logDebug("TA") << "<--- NEXT PA from fringe:"
                                << "\n"
                                << *curPa << "--->";

        if (curPa->isGoal()) {
            goal = curPa;
        } else {
            Logging::logDebug("TA") << "<--- TA: BEFORE fringe exp:" << _fringe << "--->";
            curPa->expand(_fringe, _pool, oldAss, _wm);
            Logging::logDebug("TA") << "<--- TA: AFTER fringe exp:\n"
                                    << "TA: fringe size " << _fringe.size() << _fringe << "--->";
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
    AgentId ownAgentId = _tm.getLocalAgentID();
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
    ;
    return out;
}

} /* namespace alica */
