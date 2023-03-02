#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class TestTracingMasterPlan : public DomainPlan
{
public:
    TestTracingMasterPlan(PlanContext& context);
};
class PreCondition1840401110297459509 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, TestTracingMasterPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, TestTracingMasterPlanUtilityFunction)

} /* namespace alica */
