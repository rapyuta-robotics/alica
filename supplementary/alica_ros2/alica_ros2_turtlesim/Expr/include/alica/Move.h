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

class UtilityFunctionMove : public BasicUtilityFunction
{
public:
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
    static std::shared_ptr<UtilityFunctionMove> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::UtilityFunctionMove::create, UtilityFunctionMove)
class MoveRunTimeCondition : public BasicCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
} /* namespace alica */
