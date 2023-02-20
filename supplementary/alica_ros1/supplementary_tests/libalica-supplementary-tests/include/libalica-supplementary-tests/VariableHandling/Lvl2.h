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

class Lvl2 : public alica::BasicPlan
{
public:
    Lvl2(alica::PlanContext& context);
    static std::unique_ptr<Lvl2> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::Lvl2::create, Lvl2)

class Lvl2UtilityFunction : public alica::BasicUtilityFunction
{
public:
    Lvl2UtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::unique_ptr<Lvl2UtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::Lvl2UtilityFunction::create, Lvl2UtilityFunction)

class Lvl2RuntimeCondition : public BasicCondition
{
public:
    // Factory method
    static std::unique_ptr<Lvl2RuntimeCondition> create(ConditionContext& context);
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
BOOST_DLL_ALIAS(alica::Lvl2RuntimeCondition::create, Lvl2RuntimeCondition)

class Lvl2RuntimeConditionConstraint : public BasicConstraint
{
public:
    void getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp);
    static std::unique_ptr<Lvl2RuntimeConditionConstraint> create(alica::ConstraintContext&);
};
BOOST_DLL_ALIAS(alica::Lvl2RuntimeConditionConstraint::create, Lvl2RuntimeConditionConstraint)

} // namespace alica
