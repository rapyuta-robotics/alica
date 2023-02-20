#include <libalica-tests/behaviours/Attack.h>
#include <memory>

namespace alica
{
Attack::Attack(BehaviourContext& context)
        : DomainBehaviour(context)
{
    this->callCounter = 0;
    this->initCounter = 0;
}
Attack::~Attack() {}
void Attack::run()
{
    callCounter++;
}
void Attack::initialiseParameters()
{
    callCounter = 0;
    initCounter++;
}

std::unique_ptr<Attack> Attack::create(alica::BehaviourContext& context)
{
    return std::make_unique<Attack>(context);
}

} /* namespace alica */
