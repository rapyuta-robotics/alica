#pragma once

#include <alica_tests/DomainBehaviour.h>
#include <boost/dll/alias.hpp>

namespace alica
{
class TestBehaviour : public DomainBehaviour
{
public:
    TestBehaviour(BehaviourContext& context);
    virtual ~TestBehaviour();
    virtual void run();
    static std::unique_ptr<TestBehaviour> create(alica::BehaviourContext& context);

protected:
    virtual void initialiseParameters();
    virtual void onTermination();
};

BOOST_DLL_ALIAS(alica::TestBehaviour::create, TestBehaviour)

} /* namespace alica */
