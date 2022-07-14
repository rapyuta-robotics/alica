#include <alica/Master2425328142973735249.h>
#include <alica/Move1889749086610694100.h>
#include <alica/UtilityFunctionCreator.h>
#include <iostream>

namespace alica
{

UtilityFunctionCreator::~UtilityFunctionCreator(){};

UtilityFunctionCreator::UtilityFunctionCreator(){};

void UtilityFunctionCreator::setLogger(IAlicaLogger& logger)
{
    _logger = &logger;
}

std::shared_ptr<BasicUtilityFunction> UtilityFunctionCreator::createUtility(int64_t utilityfunctionConfId)
{
    switch (utilityfunctionConfId) {
    case 1889749086610694100:
        return std::make_shared<UtilityFunction1889749086610694100>(*_logger);
        break;
    case 2425328142973735249:
        return std::make_shared<UtilityFunction2425328142973735249>(*_logger);
        break;
    default:
        std::cerr << "UtilityFunctionCreator: Unknown utility requested: " << utilityfunctionConfId << std::endl;
        throw new std::exception();
        break;
    }
}

} // namespace alica
