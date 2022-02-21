#include "UtilityFunctionCreator.h"
#include "ActionServerExample2379894799421542548.h"
#include "ActionServerExampleMaster2369418759245288160.h"
#include "DummyImplementation4126421719858579722.h"
#include "GSolver/GSolverMaster1417423751087.h"
#include "GSolver/GSolverTestPlan1417423757243.h"
#include "ProblemModule/ProbBuildingLevel11479557378264.h"
#include "ProblemModule/ProbBuildingLevel1_11479557664989.h"
#include "ProblemModule/ProblemBuildingMaster1479556022226.h"
#include "ProblemModule/QueryPlan11479556074049.h"
#include "ProblemModule/QueryPlan21479718449392.h"
#include "VariableHandling/Lvl11524452759599.h"
#include "VariableHandling/Lvl21524452793378.h"
#include "VariableHandling/Lvl31524452836022.h"
#include "VariableHandling/VHMaster1524452721452.h"
#include <iostream>

namespace alica
{

UtilityFunctionCreator::~UtilityFunctionCreator() {}

UtilityFunctionCreator::UtilityFunctionCreator() {}

std::shared_ptr<BasicUtilityFunction> UtilityFunctionCreator::createUtility(int64_t utilityfunctionConfId)
{
    switch (utilityfunctionConfId) {
    case 1417423751087:
        return std::make_shared<UtilityFunction1417423751087>();
        break;
    case 1417423757243:
        return std::make_shared<UtilityFunction1417423757243>();
        break;
    case 1479556022226:
        return std::make_shared<UtilityFunction1479556022226>();
        break;
    case 1479556074049:
        return std::make_shared<UtilityFunction1479556074049>();
        break;
    case 1479557378264:
        return std::make_shared<UtilityFunction1479557378264>();
        break;
    case 1479557664989:
        return std::make_shared<UtilityFunction1479557664989>();
        break;
    case 1479718449392:
        return std::make_shared<UtilityFunction1479718449392>();
        break;
    case 1524452721452:
        return std::make_shared<UtilityFunction1524452721452>();
        break;
    case 1524452759599:
        return std::make_shared<UtilityFunction1524452759599>();
        break;
    case 1524452793378:
        return std::make_shared<UtilityFunction1524452793378>();
        break;
    case 1524452836022:
        return std::make_shared<UtilityFunction1524452836022>();
        break;
    case 2369418759245288160:
        return std::make_shared<UtilityFunction2369418759245288160>();
        break;
    case 2379894799421542548:
        return std::make_shared<UtilityFunction2379894799421542548>();
        break;
    case 4126421719858579722:
        return std::make_shared<UtilityFunction4126421719858579722>();
        break;
    default:
        std::cerr << "UtilityFunctionCreator: Unknown utility requested: " << utilityfunctionConfId << std::endl;
        throw new std::exception();
        break;
    }
}

} // namespace alica
