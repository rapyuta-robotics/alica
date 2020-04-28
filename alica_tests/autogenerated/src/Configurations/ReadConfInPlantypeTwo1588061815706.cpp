#include "Configurations/ReadConfInPlantypeTwo1588061815706.h"
/*PROTECTED REGION ID(eph1588061815706) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:ReadConfInPlantypeTwo
/**
 */
std::shared_ptr<UtilityFunction> UtilityFunction1588061815706::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1588061815706) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}
} // namespace alica
