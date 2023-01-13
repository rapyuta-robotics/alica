#pragma once

#include "engine/BasicPlan.h"
#include "engine/BasicUtilityFunction.h"
#include <boost/dll/alias.hpp>

namespace alica
{

class Move : public BasicPlan
{
public:
    Move(PlanContext& context);
    virtual ~Move();
    // Factory method
    static std::unique_ptr<Move> create(PlanContext& context) { return std::unique_ptr<Move>(new Move(context)); }
};

BOOST_DLL_ALIAS(alica::Move::create, Move)

class MoveUtilityFunction : public BasicUtilityFunction
{
public:
    MoveUtilityFunction() = default;
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan) override;
    // Factory method
    static std::shared_ptr<MoveUtilityFunction> create(UtilityFunctionContext& context)
    {
        (void) context;
        return std::make_shared<MoveUtilityFunction>();
    }
};
BOOST_DLL_ALIAS(alica::MoveUtilityFunction::create, MoveUtilityFunction)
} /* namespace alica */
