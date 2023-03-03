#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicBehaviour.h>

namespace alica_test
{
class SchedWM;
}

namespace alica
{
class BehBAA : public BasicBehaviour
{
public:
    BehBAA(BehaviourContext& context);
    virtual ~BehBAA();
    virtual void run();
    static std::unique_ptr<BehBAA> create(alica::BehaviourContext& context);

    static int runCount;

protected:
    virtual void initialiseParameters();
    virtual void onTermination();

private:
    std::shared_ptr<alica_test::SchedWM> _wm;
};
BOOST_DLL_ALIAS(alica::BehBAA::create, BehBAA)
} /* namespace alica */
