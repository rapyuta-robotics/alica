#include "ConstraintCreator.h"

#include "constraints/ActionServerExample2379894799421542548Constraints.h"
#include "constraints/ActionServerExampleMaster2369418759245288160Constraints.h"
#include "constraints/DummyImplementation4126421719858579722Constraints.h"

#include <iostream>

namespace alica
{

ConstraintCreator::ConstraintCreator() {}

ConstraintCreator::~ConstraintCreator() {}

std::shared_ptr<BasicConstraint> ConstraintCreator::createConstraint(int64_t constraintConfId)
{
    switch (constraintConfId) {
    default:
        std::cerr << "ConstraintCreator: Unknown constraint requested: " << constraintConfId << std::endl;
        throw new std::exception();
        break;
    }
}

} // namespace alica
