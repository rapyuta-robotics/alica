#include <alica_ros_turtlesim/Master2425328142973735249.h>
#include <alica_ros_turtlesim/Move1889749086610694100.h>
#include <alica_ros_turtlesim/UtilityFunctionCreator.h>
#include <iostream>

namespace alica
{

UtilityFunctionCreator::~UtilityFunctionCreator() {}

UtilityFunctionCreator::UtilityFunctionCreator() {}

std::shared_ptr<BasicUtilityFunction> UtilityFunctionCreator::createUtility(int64_t utilityfunctionConfId)
{
    switch (utilityfunctionConfId) {
    case 1889749086610694100:
        return std::make_shared<UtilityFunction1889749086610694100>();
        break;
    case 2425328142973735249:
        return std::make_shared<UtilityFunction2425328142973735249>();
        break;
    default:
        std::cerr << "UtilityFunctionCreator: Unknown utility requested: " << utilityfunctionConfId << std::endl;
        throw new std::exception();
        break;
    }
}

} // namespace alica
