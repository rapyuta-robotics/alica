#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicBehaviour.h>

namespace alica
{
class TestParameterPassingBehaviour : public BasicBehaviour
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
