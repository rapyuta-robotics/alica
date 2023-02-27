#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
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
class PlanBA1629895873188 : public DomainPlan
{
public:
    PlanBA1629895873188(PlanContext& context);
    virtual ~PlanBA1629895873188();

protected:
    virtual void onInit() override;
    virtual void onTerminate() override;

private:
    std::shared_ptr<alica_test::SchedWM> _wm;
};

class UtilityFunction1629895873188 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, PlanBA1629895873188)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, PlanBA1629895873188UtilityFunction)
} /* namespace alica */
