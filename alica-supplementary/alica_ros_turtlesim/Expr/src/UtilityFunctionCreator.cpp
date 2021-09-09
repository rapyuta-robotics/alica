#include "UtilityFunctionCreator.h"
#include "Master1542881176278.h"
#include "Move1542882005838.h"
#include <iostream>

namespace alica
{

UtilityFunctionCreator::~UtilityFunctionCreator() {}

UtilityFunctionCreator::UtilityFunctionCreator() {}

std::shared_ptr<BasicUtilityFunction> UtilityFunctionCreator::createUtility(int64_t utilityfunctionConfId)
{
    switch (utilityfunctionConfId) {
    case 1542881176278:
        return std::make_shared<UtilityFunction1542881176278>();
        break;
    case 1542882005838:
        return std::make_shared<UtilityFunction1542882005838>();
        break;
    default:
        std::cerr << "UtilityFunctionCreator: Unknown utility requested: " << utilityfunctionConfId << std::endl;
        throw new std::exception();
        break;
    }
}

} // namespace alica
