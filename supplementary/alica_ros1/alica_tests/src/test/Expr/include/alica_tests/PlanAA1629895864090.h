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
class PlanAA1629895864090 : public DomainPlan
{
public:
    PlanAA1629895864090(PlanContext& context);
    virtual ~PlanAA1629895864090();

protected:
    virtual void onInit() override;
    virtual void onTerminate() override;

private:
    std::shared_ptr<alica_test::SchedWM> _wm;
};

class UtilityFunction1629895864090 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, PlanAA1629895864090)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, PlanAA1629895864090UtilityFunction)
} /* namespace alica */
