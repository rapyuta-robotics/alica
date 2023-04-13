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

class Lvl3 : public alica::BasicPlan
{
public:
    Lvl3(alica::PlanContext& context);
    static std::unique_ptr<Lvl3> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::Lvl3::create, Lvl3)

class Lvl3UtilityFunction : public alica::BasicUtilityFunction
{
public:
    Lvl3UtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<Lvl3UtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::Lvl3UtilityFunction::create, Lvl3UtilityFunction)

class Lvl3RuntimeCondition : public BasicCondition
{
public:
    static std::shared_ptr<Lvl3RuntimeCondition> create(ConditionContext& context);
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
BOOST_DLL_ALIAS(alica::Lvl3RuntimeCondition::create, Lvl3RuntimeCondition)

class Lvl3RuntimeConditionConstraint : public BasicConstraint
{
public:
    void getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp);
    static std::shared_ptr<Lvl3RuntimeConditionConstraint> create(alica::ConstraintContext&);
};
BOOST_DLL_ALIAS(alica::Lvl3RuntimeConditionConstraint::create, Lvl3RuntimeConditionConstraint)

} // namespace alica
