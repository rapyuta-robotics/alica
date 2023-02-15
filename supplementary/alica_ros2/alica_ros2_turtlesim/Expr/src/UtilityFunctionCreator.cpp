#include <alica/Master.h>
#include <alica/Move.h>
#include <alica/UtilityFunctionCreator.h>
#include <iostream>

namespace alica
{

UtilityFunctionCreator::~UtilityFunctionCreator() {}

UtilityFunctionCreator::UtilityFunctionCreator() {}

std::shared_ptr<BasicUtilityFunction> UtilityFunctionCreator::createUtility(int64_t utilityfunctionConfId, UtilityFunctionContext& context)
{
    switch (utilityfunctionConfId) {
    case 1889749086610694100:
        return std::make_shared<UtilityFunctionMove>();
        break;
    case 2425328142973735249:
        return std::make_shared<UtilityFunctionMaster>();
        break;
    default:
        std::cerr << "UtilityFunctionCreator: Unknown utility requested: " << utilityfunctionConfId << std::endl;
        throw new std::exception();
        break;
    }
}

} // namespace alica
