#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicCondition.h>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace turtlesim
{
class Move : public alica::BasicPlan
{
public:
    Move(alica::PlanContext& context);
    virtual ~Move();
    static std::unique_ptr<Move> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(turtlesim::Move::create, Move)

class MoveUtilityFunction : public alica::BasicUtilityFunction
{
public:
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan);
    static std::shared_ptr<MoveUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(turtlesim::MoveUtilityFunction::create, MoveUtilityFunction)

class CircleRuntimeCondition : public alica::BasicCondition
{
    bool evaluate(std::shared_ptr<alica::RunningPlan> rp, const alica::Blackboard* gb);

public:
    static std::shared_ptr<CircleRuntimeCondition> create(alica::ConditionContext&);
};
BOOST_DLL_ALIAS(turtlesim::CircleRuntimeCondition::create, CircleRuntimeCondition)
} /* namespace turtlesim */
