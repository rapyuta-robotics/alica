#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicCondition.h>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class Move : public BasicPlan
{
public:
    Move(PlanContext& context);
    virtual ~Move();
    static std::unique_ptr<Move> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::Move::create, Move)

class MoveUtilityFunction : public BasicUtilityFunction
{
public:
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
    static std::shared_ptr<MoveUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::MoveUtilityFunction::create, MoveUtilityFunction)

class CircleRuntimeCondition : public BasicCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);

public:
    static std::shared_ptr<CircleRuntimeCondition> create(alica::ConditionContext&);
};
BOOST_DLL_ALIAS(alica::CircleRuntimeCondition::create, CircleRuntimeCondition)
} /* namespace alica */
