#include <alica_tests/ConstraintCreator.h>

#include <alica_tests/constraints/AttackPlan1402488634525Constraints.h>
#include <alica_tests/constraints/ConstraintTestPlan1414068524245Constraints.h>
#include <alica_tests/constraints/GoalPlan1402488870347Constraints.h>

#include <iostream>

namespace alica
{

ConstraintCreator::ConstraintCreator() {}

ConstraintCreator::~ConstraintCreator() {}

std::shared_ptr<BasicConstraint> ConstraintCreator::createConstraint(int64_t constraintConfId)
{
    switch (constraintConfId) {
    case 1403773741874:
        return std::make_shared<Constraint1403773741874>();
        break;
    case 1414068566297:
        return std::make_shared<Constraint1414068566297>();
        break;
    default:
        std::cerr << "ConstraintCreator: Unknown constraint requested: " << constraintConfId << std::endl;
        throw new std::exception();
        break;
    }
}

} // namespace alica
