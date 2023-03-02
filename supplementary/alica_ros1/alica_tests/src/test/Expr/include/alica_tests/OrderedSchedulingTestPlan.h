#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class OrderedSchedulingTestPlan : public DomainPlan
{
public:
    OrderedSchedulingTestPlan(PlanContext& context);
};
class PreCondition1629895758612 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition1629895768182 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, OrderedSchedulingTestPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, OrderedSchedulingTestPlanUtilityFunction)
} /* namespace alica */
