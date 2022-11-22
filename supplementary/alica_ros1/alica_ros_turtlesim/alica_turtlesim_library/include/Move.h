#pragma once

#include "engine/BasicPlan.h"
#include "world_model.hpp"
#include <boost/dll/alias.hpp>

namespace alica
{

class Move : public BasicPlan
{
public:
    Move(PlanContext& context);
    virtual ~Move();
    // Factory method
    static std::unique_ptr<Move> create(PlanContext& context)
    {
        auto out = std::unique_ptr<Move>(new Move(context));
        out->getBlackboard()->impl().set("turtlesim::worldmodel", turtlesim::ALICATurtleWorldModel::wmInstance_);
        return out;
    }
};

BOOST_DLL_ALIAS(alica::Move::create, Move)
} /* namespace alica */
