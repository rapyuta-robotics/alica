#include "UtilityFunctionCreator.h"
#include "ActionServerExample2379894799421542548.h"
#include "ActionServerExampleMaster2369418759245288160.h"
#include "DummyImplementation4126421719858579722.h"
#include <iostream>

namespace alica
{

UtilityFunctionCreator::~UtilityFunctionCreator() {}

UtilityFunctionCreator::UtilityFunctionCreator() {}

std::shared_ptr<BasicUtilityFunction> UtilityFunctionCreator::createUtility(int64_t utilityfunctionConfId)
{
    switch (utilityfunctionConfId) {
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
