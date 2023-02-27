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
class PlanB1629895853508 : public DomainPlan
{
public:
    PlanB1629895853508(PlanContext& context);
    virtual ~PlanB1629895853508();

protected:
    virtual void onInit() override;
    virtual void onTerminate() override;

private:
    std::shared_ptr<alica_test::SchedWM> _wm;
};

class UtilityFunction1629895853508 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, PlanB1629895853508)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, PlanB1629895853508UtilityFunction)
} /* namespace alica */
