#pragma once

#include <boost/dll/alias.hpp>
#include <engine/AlicaClock.h>
#include <engine/BasicBehaviour.h>

namespace alica
{
class TestRepeatedRunsBeh : public BasicBehaviour
{
public:
    TestRepeatedRunsBeh(BehaviourContext& context);
    virtual void run();
    static std::unique_ptr<TestRepeatedRunsBeh> create(alica::BehaviourContext& context);

private:
    int64_t _callCounter;
    AlicaTime _start;
    AlicaTime _end;
    int _frequency;
};
BOOST_DLL_ALIAS(alica::TestRepeatedRunsBeh::create, TestRepeatedRunsBeh)
} /* namespace alica */
