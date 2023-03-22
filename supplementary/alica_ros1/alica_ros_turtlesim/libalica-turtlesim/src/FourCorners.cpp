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

std::shared_ptr<alica::UtilityFunction> FourCornersUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    // contribution weightage in utilty score for similar assignments
    double similarityWeight = 0.5;
    // in this case priorityWeight is 0 as all agents role are same.
    // it only comes into account when the role of agent differs and we have priority of each task based on agent role
    double priorityWeight = 0;
    double customWeight = 0.5;
    std::shared_ptr<alica::UtilityFunction> function = std::make_shared<alica::UtilityFunction>(priorityWeight, similarityWeight, plan);
    FourCornersSummand* summand = new FourCornersSummand(customWeight);
    summand->addEntryPoint(plan->getEntryPointByID(599427197773116147));
    summand->addEntryPoint(plan->getEntryPointByID(919409193351805481));
    summand->addEntryPoint(plan->getEntryPointByID(1002169119911528732));
    summand->addEntryPoint(plan->getEntryPointByID(3975697190021470017));
    function->editUtilSummands().emplace_back(summand);
    // final equation for utiltiy will be
    //  uility = priorityWeight*prio(assignment) + similarityWeight*sim(assignment, oldAssignment) + customWeight*summand(assignment)
    //  uility threshold is set to 1 check FourCorner.pml
    //  if summand(assignment) returns 0 that means, the current assignment is not good and for this solution to be accepted it has to be very similar to the
    //  oldAssignment. If current assignment is better but differs too much from the oldAssignment then utility score will not cross threshold and assignment
    //  will be finally rejected. At first when there is no assignment only good assignment will be selected and on subsequent iterations new assignments will
    //  be searched, which are better and similar to already selected assignment. similarityWeight helps converge the solution faster as we are not greedy in
    //  accepting better solution, in this case if some agents already found their task we mostly don't disturb them even if any new agent comes up and is
    //  better match for that task. This also helps alica internally resolve conflicts if two agetns want same task.
    return function;
}

} /* namespace turtlesim */
