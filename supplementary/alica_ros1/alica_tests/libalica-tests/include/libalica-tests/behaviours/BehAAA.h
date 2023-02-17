#pragma once

#include <alica_tests/DomainBehaviour.h>
#include <atomic>
#include <boost/dll/alias.hpp>

namespace alica_test
{
class SchedWM;
}

namespace alica
{
class BehAAA : public DomainBehaviour
{
public:
    BehAAA(BehaviourContext& context);
    virtual ~BehAAA();
    virtual void run();
    static std::unique_ptr<BehAAA> create(alica::BehaviourContext& context);

    static int runCount;

protected:
    virtual void initialiseParameters();
    virtual void onTermination();

private:
    std::atomic<bool> _inRunContext;
    std::shared_ptr<alica_test::SchedWM> _wm;
};
BOOST_DLL_ALIAS(alica::BehAAA::create, BehAAA)
} /* namespace alica */
