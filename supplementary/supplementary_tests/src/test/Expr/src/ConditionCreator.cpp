#include <supplementary_tests/ConditionCreator.h>
#include <supplementary_tests/GSolver/GSolverMaster1417423751087.h>
#include <supplementary_tests/GSolver/GSolverTestPlan1417423757243.h>
#include <supplementary_tests/GSolver/SolverTestBehaviour1417424455986.h>
#include <supplementary_tests/ProblemModule/ProbBuildingLevel11479557378264.h>
#include <supplementary_tests/ProblemModule/ProbBuildingLevel1_11479557664989.h>
#include <supplementary_tests/ProblemModule/ProblemBuildingMaster1479556022226.h>
#include <supplementary_tests/ProblemModule/QueryBehaviour11479556104511.h>
#include <supplementary_tests/ProblemModule/QueryPlan11479556074049.h>
#include <supplementary_tests/ProblemModule/QueryPlan21479718449392.h>
#include <supplementary_tests/VariableHandling/Lvl11524452759599.h>
#include <supplementary_tests/VariableHandling/Lvl21524452793378.h>
#include <supplementary_tests/VariableHandling/Lvl31524452836022.h>
#include <supplementary_tests/VariableHandling/VHMaster1524452721452.h>

namespace alica
{

ConditionCreator::ConditionCreator() {}
ConditionCreator::~ConditionCreator() {}

std::shared_ptr<BasicCondition> ConditionCreator::createConditions(ConditionContext& context)
{
    int64_t conditionConfId = context.conditionConfId;
    switch (conditionConfId) {
    case 1417424512343:
        return std::make_shared<RunTimeCondition1417424512343>();
        break;
    case 1479556084493:
        return std::make_shared<RunTimeCondition1479556084493>();
        break;
    case 1524452937477:
        return std::make_shared<RunTimeCondition1524452937477>();
        break;
    case 1524453266123:
        return std::make_shared<RunTimeCondition1524453266123>();
        break;
    case 1524453470580:
        return std::make_shared<RunTimeCondition1524453470580>();
        break;
    case 1524463006078:
        return std::make_shared<RunTimeCondition1524463006078>();
        break;
    default:
        std::cerr << "ConditionCreator: Unknown condition id requested: " << conditionConfId << std::endl;
        throw new std::exception();
        break;
    }
}
} // namespace alica
