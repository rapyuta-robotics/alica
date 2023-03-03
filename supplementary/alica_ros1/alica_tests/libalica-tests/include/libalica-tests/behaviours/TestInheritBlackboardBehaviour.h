#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicBehaviour.h>

namespace alica
{
class TestInheritBlackboardBehaviour : public BasicBehaviour
{
public:
    TestInheritBlackboardBehaviour(BehaviourContext& context);
    virtual ~TestInheritBlackboardBehaviour();
    virtual void run();
    static std::unique_ptr<TestInheritBlackboardBehaviour> create(alica::BehaviourContext& context);

protected:
    virtual void initialiseParameters();
};
BOOST_DLL_ALIAS(alica::TestInheritBlackboardBehaviour::create, TestInheritBlackboardBehaviour)
} /* namespace alica */
