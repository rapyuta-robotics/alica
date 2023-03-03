#pragma once

#include <alica_tests/util/AlicaTestsPlan.h>
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
class PlanAA : public AlicaTestsPlan<PlanAA>
{
public:
    PlanAA(PlanContext& context);

protected:
    virtual void onInit() override;
    virtual void onTerminate() override;

private:
    std::shared_ptr<alica_test::SchedWM> _wm;
};

BOOST_DLL_ALIAS(alica::PlanAA::create, PlanAA)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, PlanAAUtilityFunction)
} /* namespace alica */
