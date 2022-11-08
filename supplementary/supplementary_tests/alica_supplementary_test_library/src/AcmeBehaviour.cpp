#include "AcmeBehaviour.h"

namespace alica
{

AcmeBehaviour::AcmeBehaviour(BehaviourContext& context)
        : BasicBehaviour(context)
{
    std::cerr << "AcmeBehaviour created" << std::endl;
}
} // namespace alica
