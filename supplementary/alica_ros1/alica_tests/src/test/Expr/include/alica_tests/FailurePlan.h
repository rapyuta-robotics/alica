#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
#include <libalica-tests/util/AlicaTestsPlan.h>

namespace alica
{
class FailurePlan : public AlicaTestsPlan<FailurePlan>
{
public:
    FailurePlan(PlanContext& context);

protected:
    virtual void onInit() override;
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

BOOST_DLL_ALIAS(alica::FailurePlan::create, FailurePlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, FailurePlanUtilityFunction)
} /* namespace alica */
