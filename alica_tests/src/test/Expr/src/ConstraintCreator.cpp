#include "ConstraintCreator.h"

#include "constraints/AttackPlan1402488634525Constraints.h"
#include "constraints/ConstraintTestPlan1414068524245Constraints.h"
#include "constraints/GoalPlan1402488870347Constraints.h"

#include <iostream>

namespace alica
{

ConstraintCreator::ConstraintCreator() {}

ConstraintCreator::~ConstraintCreator() {}

std::shared_ptr<BasicConstraint> ConstraintCreator::createConstraint(int64_t constraintConfId)
{
    switch (constraintConfId) {
    case 1402489460549:
        return std::make_shared<Constraint1402489460549>();
        break;
    case 1402489462088:
        return std::make_shared<Constraint1402489462088>();
        break;
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
