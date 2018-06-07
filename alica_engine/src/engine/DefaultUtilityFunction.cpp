#include "engine/DefaultUtilityFunction.h"

namespace alica
{

UtilityFunction DefaultUtilityFunction(const Plan* plan)
{
    return UtilityFunction(1.0, 0.0, plan);
}

} /* namespace alica */
