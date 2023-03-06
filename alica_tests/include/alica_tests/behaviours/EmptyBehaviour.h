#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicBehaviour.h>

namespace alica
{
class EmptyBehaviour : public BasicBehaviour
{
public:
    EmptyBehaviour(BehaviourContext& context);
    virtual ~EmptyBehaviour();
    virtual void run();
    static std::unique_ptr<EmptyBehaviour> create(alica::BehaviourContext& context);

protected:
    virtual void initialiseParameters();
};
BOOST_DLL_ALIAS(alica::EmptyBehaviour::create, EmptyBehaviour)
} /* namespace alica */
