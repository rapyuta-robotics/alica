#include <alica_tests/behaviours/Tackle.h>
#include <memory>

namespace alica
{
Tackle::Tackle(BehaviourContext& context)
        : BasicBehaviour(context)
{
}
Tackle::~Tackle() {}
void Tackle::run() {}
void Tackle::initialiseParameters() {}
std::unique_ptr<Tackle> Tackle::create(alica::BehaviourContext& context)
{
    return std::make_unique<Tackle>(context);
}
} /* namespace alica */
