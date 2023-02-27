#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class FailurePlan631515556091266493 : public DomainPlan
{
public:
    FailurePlan631515556091266493(PlanContext& context);
    virtual ~FailurePlan631515556091266493();

protected:
    virtual void onInit() override;
};

class UtilityFunction631515556091266493 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition4351457352348187886 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition2038762164340314344 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, FailurePlan631515556091266493)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, FailurePlan631515556091266493UtilityFunction)
} /* namespace alica */
