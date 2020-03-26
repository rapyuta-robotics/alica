#include "ConditionCreator.h"
#include "GSolver/GSolverMaster1417423751087.h"
#include "GSolver/GSolverTestPlan1417423757243.h"
#include "GSolver/SolverTestBehaviour1417424455986.h"
#include "ProblemModule/ProbBuildingLevel11479557378264.h"
#include "ProblemModule/ProbBuildingLevel1_11479557664989.h"
#include "ProblemModule/ProblemBuildingMaster1479556022226.h"
#include "ProblemModule/QueryBehaviour11479556104511.h"
#include "ProblemModule/QueryPlan11479556074049.h"
#include "ProblemModule/QueryPlan21479718449392.h"
#include "VariableHandling/Lvl11524452759599.h"
#include "VariableHandling/Lvl21524452793378.h"
#include "VariableHandling/Lvl31524452836022.h"
#include "VariableHandling/VHMaster1524452721452.h"

namespace alica
{

ConditionCreator::ConditionCreator() {}
ConditionCreator::~ConditionCreator() {}

std::shared_ptr<BasicCondition> ConditionCreator::createConditions(long conditionConfId)
{
    switch (conditionConfId) {
    case 1417424512343:
        return std::make_shared<RunTimeCondition1417424512343>();
        break;
    case 1479556084493:
        return std::make_shared<RunTimeCondition1479556084493>();
        break;
    case 1479557592662:
        return std::make_shared<PreCondition1479557592662>();
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
    case 1524453491764:
        return std::make_shared<PreCondition1524453491764>();
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
