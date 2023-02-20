#pragma once

#include <engine/BasicCondition.h>
#include <engine/BasicConstraint.h>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica
{
class ProblemDescriptor;

class Lvl1 : public alica::BasicPlan
{
public:
    Lvl1(alica::PlanContext& context);
    static std::unique_ptr<Lvl1> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::Lvl1::create, Lvl1)

class Lvl1UtilityFunction : public alica::BasicUtilityFunction
{
public:
    Lvl1UtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::unique_ptr<Lvl1UtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::Lvl1UtilityFunction::create, Lvl1UtilityFunction)

class Lvl1RuntimeCondition : public BasicCondition
{
public:
    // Factory method
    static std::unique_ptr<Lvl1RuntimeCondition> create(ConditionContext& context);
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
BOOST_DLL_ALIAS(alica::Lvl1RuntimeCondition::create, Lvl1RuntimeCondition)

class Lvl1RuntimeConditionConstraint : public BasicConstraint
{
public:
    void getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp);
    static std::unique_ptr<Lvl1RuntimeConditionConstraint> create(alica::ConstraintContext&);
};
BOOST_DLL_ALIAS(alica::Lvl1RuntimeConditionConstraint::create, Lvl1RuntimeConditionConstraint)
} // namespace alica
