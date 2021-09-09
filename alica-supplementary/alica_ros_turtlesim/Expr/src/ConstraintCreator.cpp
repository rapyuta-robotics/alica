#include "ConstraintCreator.h"

#include "Behaviours/constraints/Go2RandomPosition1542881969548Constraints.h"
#include "Behaviours/constraints/GoTo1544160969061Constraints.h"
#include "constraints/Master1542881176278Constraints.h"
#include "constraints/Move1542882005838Constraints.h"

#include <iostream>

namespace alica
{

ConstraintCreator::ConstraintCreator() {}

ConstraintCreator::~ConstraintCreator() {}

std::shared_ptr<BasicConstraint> ConstraintCreator::createConstraint(int64_t constraintConfId)
{
    switch (constraintConfId) {
    case 1543284793605:
        return std::make_shared<Constraint1543284793605>();
        break;
    default:
        std::cerr << "ConstraintCreator: Unknown constraint requested: " << constraintConfId << std::endl;
        throw new std::exception();
        break;
    }
}

} // namespace alica
