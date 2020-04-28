#include "Configurations/ReadConfInPlantypeOne1588061801734.h"
/*PROTECTED REGION ID(eph1588061801734) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:ReadConfInPlantypeOne
/**
 */
std::shared_ptr<UtilityFunction> UtilityFunction1588061801734::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1588061801734) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}
} // namespace alica
