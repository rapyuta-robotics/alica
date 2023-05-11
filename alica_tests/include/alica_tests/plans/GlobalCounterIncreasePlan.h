#pragma once

#include <alica_tests/util/AlicaTestsPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>

namespace alica
{
class GlobalCounterIncreasePlan : public AlicaTestsPlan<GlobalCounterIncreasePlan>
{
public:
    GlobalCounterIncreasePlan(PlanContext& context);

protected:
    virtual void onInit() override;
    virtual void onTerminate() override;
};

BOOST_DLL_ALIAS(alica::GlobalCounterIncreasePlan::create, GlobalCounterIncreasePlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, GlobalCounterIncreasePlanUtilityFunction)
} /* namespace alica */
