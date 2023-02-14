#pragma once

#include <alica_tests/DomainBehaviour.h>
#include <boost/dll/alias.hpp>

namespace alica
{
class TestInheritBlackboardBehaviour : public DomainBehaviour
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
