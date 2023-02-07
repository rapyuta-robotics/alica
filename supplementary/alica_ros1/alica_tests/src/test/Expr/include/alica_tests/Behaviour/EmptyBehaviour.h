#pragma once

#include <alica_tests/DomainBehaviour.h>
#include <boost/dll/alias.hpp>

namespace alica
{
class EmptyBehaviour : public DomainBehaviour
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
