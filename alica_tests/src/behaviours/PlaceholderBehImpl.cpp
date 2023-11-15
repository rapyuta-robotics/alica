#include <alica_tests/behaviours/PlaceholderBehImpl.h>
#include <memory>

namespace alica
{

PlaceholderBehImpl::PlaceholderBehImpl(BehaviourContext& context)
        : BasicBehaviour(context)
{
}

std::unique_ptr<PlaceholderBehImpl> PlaceholderBehImpl::create(alica::BehaviourContext& context)
{
    return std::make_unique<PlaceholderBehImpl>(context);
}

} /* namespace alica */
