#pragma once

#include <engine/BasicCondition.h>
#include <engine/BasicConstraint.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace turtlesim
{

class CircleRuntimeCondition : public alica::BasicCondition
{
public:
    bool evaluate(std::shared_ptr<alica::RunningPlan> rp, const alica::Blackboard* globalBlackboard) override;
    static std::shared_ptr<CircleRuntimeCondition> create(alica::ConditionContext&);
};
BOOST_DLL_ALIAS(turtlesim::CircleRuntimeCondition::create, CircleRuntimeCondition)

class CircleRuntimeConditionConstraint : public alica::BasicConstraint
{
public:
    void getConstraint(std::shared_ptr<alica::ProblemDescriptor> c, std::shared_ptr<alica::RunningPlan> rp) override;
    static std::shared_ptr<CircleRuntimeConditionConstraint> create(alica::ConstraintContext&);
};
BOOST_DLL_ALIAS(turtlesim::CircleRuntimeConditionConstraint::create, CircleRuntimeConditionConstraint)

} // namespace turtlesim
