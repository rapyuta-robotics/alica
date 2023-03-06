#pragma once

#include <alica_tests/util/AlicaTestsPlan.h>
#include <atomic>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica_test
{
class SchedWM;
}

namespace alica
{
class PlanA : public AlicaTestsPlan<PlanA>
{
public:
    PlanA(PlanContext& context);

protected:
    virtual void run() override;
    virtual void onInit() override;
    virtual void onTerminate() override;

private:
    std::atomic<bool> _inRunContext;
    std::shared_ptr<alica_test::SchedWM> _wm;
};

BOOST_DLL_ALIAS(alica::PlanA::create, PlanA)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, PlanAUtilityFunction)
} /* namespace alica */
