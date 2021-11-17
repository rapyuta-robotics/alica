#include "ConstraintCreator.h"

#include "constraints/Go2RandomPosition4085572422059465423Constraints.h"
#include "constraints/GoTo4054297592460872311Constraints.h"
#include "constraints/Master2425328142973735249Constraints.h"
#include "constraints/Move1889749086610694100Constraints.h"

#include <iostream>

namespace alica
{

ConstraintCreator::ConstraintCreator() {}

ConstraintCreator::~ConstraintCreator() {}

std::shared_ptr<BasicConstraint> ConstraintCreator::createConstraint(int64_t constraintConfId)
{
    switch (constraintConfId) {
    case 1288817888979746811:
        return std::make_shared<Constraint1288817888979746811>();
        break;
    default:
        std::cerr << "ConstraintCreator: Unknown constraint requested: " << constraintConfId << std::endl;
        throw new std::exception();
        break;
    }
}

} // namespace alica
