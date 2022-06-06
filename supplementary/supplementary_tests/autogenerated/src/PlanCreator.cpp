#include "engine/BasicPlan.h"
#include <supplementary_tests/GSolver/GSolverMaster1417423751087.h>
#include <supplementary_tests/GSolver/GSolverTestPlan1417423757243.h>
#include <supplementary_tests/PlanCreator.h>
#include <supplementary_tests/ProblemModule/ProbBuildingLevel11479557378264.h>
#include <supplementary_tests/ProblemModule/ProbBuildingLevel1_11479557664989.h>
#include <supplementary_tests/ProblemModule/ProblemBuildingMaster1479556022226.h>
#include <supplementary_tests/ProblemModule/QueryPlan11479556074049.h>
#include <supplementary_tests/ProblemModule/QueryPlan21479718449392.h>
#include <supplementary_tests/VariableHandling/Lvl11524452759599.h>
#include <supplementary_tests/VariableHandling/Lvl21524452793378.h>
#include <supplementary_tests/VariableHandling/Lvl31524452836022.h>
#include <supplementary_tests/VariableHandling/VHMaster1524452721452.h>

namespace alica
{

PlanCreator::PlanCreator() {}

PlanCreator::~PlanCreator() {}

std::unique_ptr<BasicPlan> PlanCreator::createPlan(int64_t planId, PlanContext& context)
{
    switch (planId) {
    case 1417423751087:
        return std::make_unique<GSolverMaster1417423751087>(context);
        break;
    case 1417423757243:
        return std::make_unique<GSolverTestPlan1417423757243>(context);
        break;
    case 1479556022226:
        return std::make_unique<ProblemBuildingMaster1479556022226>(context);
        break;
    case 1479556074049:
        return std::make_unique<QueryPlan11479556074049>(context);
        break;
    case 1479557378264:
        return std::make_unique<ProbBuildingLevel11479557378264>(context);
        break;
    case 1479557664989:
        return std::make_unique<ProbBuildingLevel1_11479557664989>(context);
        break;
    case 1479718449392:
        return std::make_unique<QueryPlan21479718449392>(context);
        break;
    case 1524452721452:
        return std::make_unique<VHMaster1524452721452>(context);
        break;
    case 1524452759599:
        return std::make_unique<Lvl11524452759599>(context);
        break;
    case 1524452793378:
        return std::make_unique<Lvl21524452793378>(context);
        break;
    case 1524452836022:
        return std::make_unique<Lvl31524452836022>(context);
        break;
    default:
        std::cerr << "PlanCreator: Unknown plan requested: " << planId << std::endl;
        throw new std::exception();
        break;
    }
}
} // namespace alica
