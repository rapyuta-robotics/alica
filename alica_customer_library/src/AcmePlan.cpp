#include "AcmePlan.h"

namespace alica
{

AcmePlan::AcmePlan(BehaviourContext& context)
        : BasicBehaviour(context)
{
    std::cerr << "AcmePlan created" << std::endl;
}
} // namespace alica
