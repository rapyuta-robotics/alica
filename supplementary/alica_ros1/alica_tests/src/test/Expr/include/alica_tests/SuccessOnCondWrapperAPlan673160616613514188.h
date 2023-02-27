#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class SuccessOnCondWrapperAPlan673160616613514188 : public DomainPlan
{
public:
    SuccessOnCondWrapperAPlan673160616613514188(PlanContext& context);
    virtual ~SuccessOnCondWrapperAPlan673160616613514188();

protected:
    virtual void onInit() override;
};

class UtilityFunction673160616613514188 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition4605367163774150375 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, SuccessOnCondWrapperAPlan673160616613514188)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, SuccessOnCondWrapperAPlan673160616613514188UtilityFunction)

} /* namespace alica */
