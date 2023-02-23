#include "RandomMoveTutorial.h"

#include <engine/DefaultUtilityFunction.h>

namespace turtlesim
{

RandomMoveTutorial::RandomMoveTutorial(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void RandomMoveTutorial::onInit()
{
    alica::LockedBlackboardRW bb(*getBlackboard());
    bb.set<double>("idletime", 5);
    bb.set<double>("xmin", 0.0);
    bb.set<double>("ymin", 0.0);
    bb.set<double>("xmax", 10.0);
    bb.set<double>("ymax", 10.0);
}

std::unique_ptr<RandomMoveTutorial> RandomMoveTutorial::create(alica::PlanContext& context)
{
    return std::make_unique<RandomMoveTutorial>(context);
}

} // namespace turtlesim
