#include "ConstraintCreator.h"

#include "GSolver/constraints/GSolverMaster1417423751087Constraints.h"
#include "GSolver/constraints/GSolverTestPlan1417423757243Constraints.h"
#include "GSolver/constraints/SolverTestBehaviour1417424455986Constraints.h"
#include "ProblemModule/constraints/ProbBuildingLevel11479557378264Constraints.h"
#include "ProblemModule/constraints/ProbBuildingLevel1_11479557664989Constraints.h"
#include "ProblemModule/constraints/ProblemBuildingMaster1479556022226Constraints.h"
#include "ProblemModule/constraints/QueryBehaviour11479556104511Constraints.h"
#include "ProblemModule/constraints/QueryPlan11479556074049Constraints.h"
#include "ProblemModule/constraints/QueryPlan21479718449392Constraints.h"
#include "VariableHandling/constraints/Lvl11524452759599Constraints.h"
#include "VariableHandling/constraints/Lvl21524452793378Constraints.h"
#include "VariableHandling/constraints/Lvl31524452836022Constraints.h"
#include "VariableHandling/constraints/VHMaster1524452721452Constraints.h"

#include <iostream>

namespace alica
{

ConstraintCreator::ConstraintCreator() {}

ConstraintCreator::~ConstraintCreator() {}

std::shared_ptr<BasicConstraint> ConstraintCreator::createConstraint(long constraintConfId)
{
    switch (constraintConfId) {
    case 1417424512343:
        return std::make_shared<Constraint1417424512343>();
        break;
    case 1479556084493:
        return std::make_shared<Constraint1479556084493>();
        break;
    case 1479557592662:
        return std::make_shared<Constraint1479557592662>();
        break;
    case 1524452937477:
        return std::make_shared<Constraint1524452937477>();
        break;
    case 1524453266123:
        return std::make_shared<Constraint1524453266123>();
        break;
    case 1524453470580:
        return std::make_shared<Constraint1524453470580>();
        break;
    case 1524453491764:
        return std::make_shared<Constraint1524453491764>();
        break;
    case 1524463006078:
        return std::make_shared<Constraint1524463006078>();
        break;
    default:
        std::cerr << "ConstraintCreator: Unknown constraint requested: " << constraintConfId << std::endl;
        throw new std::exception();
        break;
    }
}

} // namespace alica
