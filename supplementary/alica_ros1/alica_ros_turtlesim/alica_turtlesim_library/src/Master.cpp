#include "Master.h"

namespace alica
{

Master::Master(PlanContext& context)
        : BasicPlan(context)
{
    std::cerr << "Master created" << std::endl;
}

void Master::onInit()
{
    LockedBlackboardRW bb(*(getBlackboard()));
    bb.registerValue("turtlesim::worldmodel", turtlesim::ALICATurtleWorldModel::wmInstance_);
}
Master::~Master() {}

} // namespace alica
