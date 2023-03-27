#include "FourCorners.h"
#include "FourCornersSummand.h"
#include "engine/USummand.h"
#include "engine/blackboard/Blackboard.h"
#include "ros/ros.h"
namespace turtlesim
{

std::shared_ptr<FourCornersUtilityFunction> FourCornersUtilityFunction::create(alica::UtilityFunctionContext& context)
{
    return std::make_shared<FourCornersUtilityFunction>(context);
}

/*
    The utility function combines three components to determine the best assignment:
    1. Priority (prio): A measure of how important the assignment is considering agent role and its priority to take up a task.
    2. Similarity (sim): A measure of how close the new assignment is to the previous assignment.
    3. Custom component (summand): A user-defined component to add extra criteria.

    Utility = priorityWeight * prio(assignment) + similarityWeight * sim(assignment, oldAssignment) + customWeight * summand(assignment)

    The utility threshold is set to 0.750 for this plan (see FourCorner.pml for details).

    If summand(assignment) returns 0, it indicates that the current assignment is not ideal. In this case, it can only be accepted if it is highly similar to
   the oldAssignment. When the current assignment is an improvement but deviates considerably from the oldAssignment, the utility score will not surpass the
   threshold, leading to the assignment's rejection.

    Initially, only suitable assignments are selected. In later iterations, the algorithm searches for new assignments that are both better and similar to the
   selected ones.

    The similarityWeight helps the solution converge faster by not being too greedy in accepting better solutions. It also helps maintain stability among agents
   who have already found their tasks. This approach also assists the alica framework in resolving conflicts if two agents want the same task.
*/
std::shared_ptr<alica::UtilityFunction> FourCornersUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    double similarityWeight = 0.5;
    // The priorityWeight is set to 0 because all agents have the same role.
    // It becomes relevant when agents have different roles, and each task has a priority based on the agent's role.
    double priorityWeight = 0;
    double customWeight = 0.5;
    std::shared_ptr<alica::UtilityFunction> function = std::make_shared<alica::UtilityFunction>(priorityWeight, similarityWeight, plan);
    FourCornersSummand* summand = new FourCornersSummand(customWeight);
    summand->addEntryPoint(plan->getEntryPointByID(599427197773116147));  // goto bottom-left
    summand->addEntryPoint(plan->getEntryPointByID(919409193351805481));  // goto top-left
    summand->addEntryPoint(plan->getEntryPointByID(1002169119911528732)); // goto bottom-right
    summand->addEntryPoint(plan->getEntryPointByID(3975697190021470017)); // goto top-right
    function->editUtilSummands().emplace_back(summand);
    return function;
}

} /* namespace turtlesim */
