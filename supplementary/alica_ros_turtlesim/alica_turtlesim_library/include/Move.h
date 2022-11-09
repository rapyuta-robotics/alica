#pragma once

#include "engine/BasicPlan.h"
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
} /* namespace alica */
