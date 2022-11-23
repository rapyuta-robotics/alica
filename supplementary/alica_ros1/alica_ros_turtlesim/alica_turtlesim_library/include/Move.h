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
    static std::unique_ptr<Move> create(PlanContext& context) { return std::make_unique<Move>(context); }

protected:
    virtual void onInit() override;
};

BOOST_DLL_ALIAS(alica::Move::create, Move)
} /* namespace alica */
