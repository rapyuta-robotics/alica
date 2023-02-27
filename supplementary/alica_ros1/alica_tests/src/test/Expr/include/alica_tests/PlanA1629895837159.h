#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
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
class PlanA1629895837159 : public DomainPlan
{
public:
    PlanA1629895837159(PlanContext& context);
    virtual ~PlanA1629895837159();

protected:
    virtual void run() override;
    virtual void onInit() override;
    virtual void onTerminate() override;

private:
    std::atomic<bool> _inRunContext;
    std::shared_ptr<alica_test::SchedWM> _wm;
};

class UtilityFunction1629895837159 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, PlanA1629895837159)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, PlanA1629895837159UtilityFunction)
} /* namespace alica */
