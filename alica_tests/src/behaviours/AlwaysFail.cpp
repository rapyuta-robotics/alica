#include <alica_tests/behaviours/AlwaysFail.h>
#include <memory>

namespace alica
{

AlwaysFail::AlwaysFail(BehaviourContext& context)
        : BasicBehaviour(context)
{
}
AlwaysFail::~AlwaysFail() {}
void AlwaysFail::run()
{
    setFailure();
}
void AlwaysFail::initialiseParameters() {}

std::unique_ptr<AlwaysFail> AlwaysFail::create(alica::BehaviourContext& context)
{
    return std::make_unique<AlwaysFail>(context);
}
} /* namespace alica */
