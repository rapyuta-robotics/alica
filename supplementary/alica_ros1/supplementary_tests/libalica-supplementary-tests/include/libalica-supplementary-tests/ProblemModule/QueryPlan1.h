#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>
#include <engine/BasicCondition.h>
#include <engine/BasicConstraint.h>

#include <boost/dll/alias.hpp>

namespace alica
{

class QueryPlan1 : public alica::BasicPlan
{
public:
    QueryPlan1(alica::PlanContext& context);
    static std::unique_ptr<QueryPlan1> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::QueryPlan1::create, QueryPlan1)

class QueryPlan1UtilityFunction : public alica::BasicUtilityFunction
{
public:
    QueryPlan1UtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::unique_ptr<QueryPlan1UtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::QueryPlan1UtilityFunction::create, QueryPlan1UtilityFunction)

class QueryPlan1RuntimeCondition : public BasicCondition
{
public:
    // Factory method
    static std::unique_ptr<QueryPlan1RuntimeCondition> create(ConditionContext& context);
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
BOOST_DLL_ALIAS(alica::QueryPlan1RuntimeCondition::create, QueryPlan1RuntimeCondition)

class QueryPlan1RuntimeConditionConstraint : public BasicConstraint
{
public:
    static std::unique_ptr<QueryPlan1RuntimeConditionConstraint> create(ConstraintContext& context);
    void getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp);
};
BOOST_DLL_ALIAS(alica::QueryPlan1RuntimeConditionConstraint::create, QueryPlan1RuntimeConditionConstraint)
} // namespace alica
