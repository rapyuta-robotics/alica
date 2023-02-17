#pragma once

#include <alica_tests/DomainBehaviour.h>
#include <boost/dll/alias.hpp>

namespace alica
{
class TestParameterPassingBehaviour : public DomainBehaviour
{
public:
    TestParameterPassingBehaviour(BehaviourContext& context);
    virtual ~TestParameterPassingBehaviour();
    virtual void run();
    static std::unique_ptr<TestParameterPassingBehaviour> create(alica::BehaviourContext& context);

protected:
    virtual void initialiseParameters();
};
BOOST_DLL_ALIAS(alica::TestParameterPassingBehaviour::create, TestParameterPassingBehaviour)
} /* namespace alica */
