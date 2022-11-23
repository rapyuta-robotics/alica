#include "Move.h"

namespace alica
{

Move::Move(PlanContext& context)
        : BasicPlan(context)
{
    std::cerr << "Move created" << std::endl;
}

void Move::onInit()
{
    LockedBlackboardRW bb(*(getBlackboard()));
    bb.registerValue("turtlesim::worldmodel", turtlesim::ALICATurtleWorldModel::wmInstance_);
}

Move::~Move() {}

} // namespace alica
