#include <alica_tests/behaviours/EmptyBehaviour.h>
#include <memory>

#include <alica/test/CounterClass.h>

namespace alica
{
EmptyBehaviour::EmptyBehaviour(BehaviourContext& context)
        : BasicBehaviour(context)
{
}
EmptyBehaviour::~EmptyBehaviour() {}
void EmptyBehaviour::run()
{
    ++CounterClass::called;
}
void EmptyBehaviour::initialiseParameters()
{
    CounterClass::called = 0;
}

std::unique_ptr<EmptyBehaviour> EmptyBehaviour::create(alica::BehaviourContext& context)
{
    return std::make_unique<EmptyBehaviour>(context);
}

} /* namespace alica */
