#include <supplementary_tests/ConstraintCreator.h>

#include <supplementary_tests/GSolver/constraints/GSolverTestPlan1417423757243Constraints.h>
#include <supplementary_tests/ProblemModule/constraints/ProblemBuildingMaster1479556022226Constraints.h>
#include <supplementary_tests/ProblemModule/constraints/QueryPlan11479556074049Constraints.h>
#include <supplementary_tests/VariableHandling/constraints/Lvl11524452759599Constraints.h>
#include <supplementary_tests/VariableHandling/constraints/Lvl21524452793378Constraints.h>
#include <supplementary_tests/VariableHandling/constraints/Lvl31524452836022Constraints.h>
#include <supplementary_tests/VariableHandling/constraints/VHMaster1524452721452Constraints.h>

#include <iostream>

namespace alica
{

ConstraintCreator::ConstraintCreator() {}

ConstraintCreator::~ConstraintCreator() {}

std::shared_ptr<BasicConstraint> ConstraintCreator::createConstraint(int64_t constraintConfId)
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
