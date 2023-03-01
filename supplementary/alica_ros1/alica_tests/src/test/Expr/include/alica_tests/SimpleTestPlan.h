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
class SimpleTestPlan : public AlicaTestsPlan<SimpleTestPlan>
{
public:
    SimpleTestPlan(PlanContext& context);

protected:
    virtual void onInit() override;
};
class SimpleTestPlanPreCondition : public DomainCondition
{
public:
    SimpleTestPlanPreCondition(ConditionContext& context)
            : DomainCondition(){};
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
    static std::unique_ptr<SimpleTestPlanPreCondition> create(ConditionContext& context) { return std::make_unique<SimpleTestPlanPreCondition>(context); };
};
class SimpleTestPlanRuntimeCondition : public DomainCondition
{
public:
    SimpleTestPlanRuntimeCondition(ConditionContext& context)
            : DomainCondition(){};
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
    static std::unique_ptr<SimpleTestPlanRuntimeCondition> create(ConditionContext& context)
    {
        return std::make_unique<SimpleTestPlanRuntimeCondition>(context);
    };
};

BOOST_DLL_ALIAS(alica::SimpleTestPlan::create, SimpleTestPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, SimpleTestPlanUtilityFunction)
BOOST_DLL_ALIAS(alica::SimpleTestPlanPreCondition::create, SimpleTestPlanPreCondition)
BOOST_DLL_ALIAS(alica::SimpleTestPlanRuntimeCondition::create, SimpleTestPlanRuntimeCondition)
} /* namespace alica */
