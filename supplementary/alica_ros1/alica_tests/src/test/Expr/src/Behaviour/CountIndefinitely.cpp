#include <alica_tests/Behaviour/CountIndefinitely.h>
#include <memory>

#include <alica/test/CounterClass.h>

namespace alica
{

CountIndefinitely::CountIndefinitely(BehaviourContext& context)
        : DomainBehaviour(context)
{
}
CountIndefinitely::~CountIndefinitely() {}
void CountIndefinitely::run()
{
    ++CounterClass::called;
}
void CountIndefinitely::initialiseParameters() {}

std::unique_ptr<CountIndefinitely> CountIndefinitely::create(alica::BehaviourContext& context)
{
    return std::make_unique<CountIndefinitely>(context);
}

} /* namespace alica */
