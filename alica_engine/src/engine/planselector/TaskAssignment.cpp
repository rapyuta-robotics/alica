#include "engine/AlicaEngine.h"
#include "engine/Assignment.h"
#include "engine/TeamObserver.h"
#include "engine/UtilityFunction.h"
#include "engine/model/Plan.h"
#include "engine/planselector/PartialAssignment.h"
#include "engine/planselector/PartialAssignmentPool.h"
#include "engine/teammanager/TeamManager.h"
#include <engine/planselector/TaskAssignment.h>

#include "engine/model/EntryPoint.h"
#include "engine/model/Task.h"

#include <supplementary/AgentID.h>

#include <alica_common_config/debug_output.h>

namespace alica
{

TaskAssignment::~TaskAssignment() {}

/**
 * Constructor of a new TaskAssignment
 * @param planList Plans to build an assignment for
 * @param paraRobots robots to build an assignment for
 * @param a bool
 */
TaskAssignment::TaskAssignment(const AlicaEngine* engine, const PlanGrp& planList, const AgentGrp& paraRobots, bool preassignOtherRobots)
        : robots(paraRobots)
        , planList(planList)
{
#ifdef EXPANSIONEVAL
    this->expansionCount = 0;
#endif
    this->to = engine->getTeamObserver();
    this->tm = engine->getTeamManager();
    // sort robot ids ascending
    std::sort(robots.begin(), robots.end(), supplementary::AgentIDComparator());
    this->fringe = vector<PartialAssignment*>();
    auto simplePlanTreeMap = to->getTeamPlanTrees();
    PartialAssignment* curPa;
    for (const Plan* curPlan : this->planList) {
        // CACHE EVALUATION DATA IN EACH USUMMAND
        curPlan->getUtilityFunction()->cacheEvalData();

        // CREATE INITIAL PARTIAL ASSIGNMENTS
        curPa = PartialAssignment::getNew(engine->getPartialAssignmentPool(), this->robots, curPlan, to->getSuccessCollection(curPlan));

        // ASSIGN PREASSIGNED OTHER ROBOTS
        if (preassignOtherRobots) {
            if (this->addAlreadyAssignedRobots(curPa, &(*simplePlanTreeMap))) {
                // revaluate this pa
                curPlan->getUtilityFunction()->updateAssignment(curPa, nullptr);
            }
        }
        this->fringe.push_back(curPa);
    }
}

/**
 * Gets the Assignment with next best utility
 * @param oldAss old Assignment
 * @return An Assignment for the plan
 */
Assignment TaskAssignment::getNextBestAssignment(const Assignment* oldAss)
{
    ALICA_DEBUG_MSG("TA: Calculating next best PartialAssignment...");
    PartialAssignment* calculatedPa = calcNextBestPartialAssignment(oldAss);

    if (calculatedPa == nullptr) {
        return Assignment();
    }

    ALICA_DEBUG_MSG("TA: ... calculated this PartialAssignment:\n" << calculatedPa->toString());

    Assignment newAss = Assignment(*calculatedPa);

    ALICA_DEBUG_MSG("TA: Return this Assignment to PS:" << newAss);

    return newAss;
}

std::string TaskAssignment::toString() const
{
    std::stringstream ss;
    ss << std::endl;
    ss << "--------------------TA:--------------------" << std::endl;
    ss << "NumRobots: " << this->robots.size() << std::endl;
    ss << "RobotIDs: ";
    for (int i = 0; i < static_cast<int>(this->robots.size()); ++i) // RobotIds
    {
        ss << this->robots[i] << " ";
    }
    ss << std::endl;
    ss << "Initial Fringe (Count " << this->fringe.size() << "):" << std::endl;
    ss << "{";
    for (int i = 0; i < static_cast<int>(this->fringe.size()); ++i) // Initial PartialAssignments
    {
        ss << this->fringe[i]->toString();
    }
    ss << "}" << std::endl;
    ss << "-------------------------------------------" << std::endl;
    ;
    return ss.str();
}

#ifdef EXPANSIONEVAL
int TaskAssignment::getExpansionCount()
{
    return expansionCount;
}

void TaskAssignment::setExpansionCount(int expansionCount)
{
    this->expansionCount = expansionCount;
}
#endif

PartialAssignment* TaskAssignment::calcNextBestPartialAssignment(const Assignment* oldAss)
{
    PartialAssignment* curPa = nullptr;
    PartialAssignment* goal = nullptr;
    while (this->fringe.size() > 0 && goal == nullptr) {
        curPa = this->fringe.at(0);
        this->fringe.erase(this->fringe.begin());
#ifdef TA_DEBUG
        std::cout << "<---" << std::endl;
        std::cout << "TA: NEXT PA from fringe:" << std::endl;
        std::cout << curPa->toString() << "--->" << std::endl;
#endif
        // Check if it is a goal
        if (curPa->isGoal()) {
            // Save the goal in result
            goal = curPa;
        }
#ifdef TA_DEBUG
        std::cout << "<---" << endl;
        std::cout << "TA: BEFORE fringe exp:" << std::endl;
        std::cout << "TA: robotID " << this->to->getOwnId() << std::endl;
        for (int i = 0; i < this->fringe.size(); i++) {
            cout << this->fringe[i]->toString();
        }
        std::cout << "--->" << std::endl;
#endif
        // Expand for the next search (maybe necessary later)
        auto newPas = curPa->expand();
#ifdef EXPANSIONEVAL
        expansionCount++;
#endif
        // Every just expanded partial assignment must get an updated utility
        for (int i = 0; i < static_cast<int>(newPas->size()); i++) {
            // Update the utility values
            auto iter = newPas->begin();
            advance(iter, i);
            (*iter)->getUtilFunc()->updateAssignment((*iter), oldAss);
            if ((*iter)->getMax() != -1) // add this partial assignment only, if all assigned robots does not have a
                                         // priority of -1 for any task
            {
                // Add to search fringe
                this->fringe.push_back((*iter));
            }
        }
        stable_sort(fringe.begin(), fringe.end(), PartialAssignment::compareTo);
#ifdef TA_DEBUG
        std::cout << "<---" << std::endl;
        std::cout << "TA: AFTER fringe exp:" << std::endl;
        std::cout << "TA: fringe size " << this->fringe.size() << std::endl;
        for (int i = 0; i < this->fringe.size(); i++) {
            cout << this->fringe[i]->toString();
        }
        std::cout << "--->" << std::endl;
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
bool TaskAssignment::addAlreadyAssignedRobots(
        PartialAssignment* pa, std::map<const supplementary::AgentID*, std::shared_ptr<SimplePlanTree>, supplementary::AgentIDComparator>* simplePlanTreeMap)
{
    const supplementary::AgentID* ownRobotId = this->tm->getLocalAgentID();
    bool haveToRevalute = false;
    std::shared_ptr<SimplePlanTree> spt = nullptr;
    for (const supplementary::AgentID* robot : robots) {
        if (*ownRobotId == *robot) {
            continue;
        }
        auto iter = simplePlanTreeMap->find(robot);
        if (iter != simplePlanTreeMap->end()) {
            spt = iter->second;
            if (pa->addIfAlreadyAssigned(spt, robot)) {
                haveToRevalute = true;
            }
        }
    }
    return haveToRevalute;
}

} /* namespace alica */
