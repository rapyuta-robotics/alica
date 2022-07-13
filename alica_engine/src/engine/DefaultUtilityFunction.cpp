#include "engine/DefaultUtilityFunction.h"

namespace alica
{

DefaultUtilityFunction::DefaultUtilityFunction(const Plan* plan, IAlicaLogger& logger)
        : UtilityFunction(1.0, 0.0, plan, logger)
{
}

} /* namespace alica */
