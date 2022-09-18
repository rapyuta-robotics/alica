#include <alica/Other.h>

namespace alica
{

/**
 * Task: DefaultTask  -> EntryPoint-ID: 2741715629576575326
 */
std::shared_ptr<UtilityFunction> UtilityFunction2425328142973735249::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(2425328142973735249) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/**
 * Task: Follower  -> EntryPoint-ID: 3277312192440194145
 * Task: Leader  -> EntryPoint-ID: 4346694000146342467
 */
std::shared_ptr<UtilityFunction> UtilityFunction1889749086610694100::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1889749086610694100) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

} // namespace alica
