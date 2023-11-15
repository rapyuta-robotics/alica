#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicBehaviour.h>

namespace alica
{
class PlaceholderBehImpl : public BasicBehaviour
{
public:
    PlaceholderBehImpl(BehaviourContext& context);
    static std::unique_ptr<PlaceholderBehImpl> create(alica::BehaviourContext& context);
};
BOOST_DLL_ALIAS(alica::PlaceholderBehImpl::create, PlaceholderBehImpl)
} /* namespace alica */
