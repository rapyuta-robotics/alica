#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace turtlesim
{

class RandomMoveTutorial : public alica::BasicPlan
{
public:
    RandomMoveTutorial(alica::PlanContext& context);
    void onInit() override;
    static std::unique_ptr<RandomMoveTutorial> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(turtlesim::RandomMoveTutorial::create, RandomMoveTutorial)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, RandomMoveTutorialUtilityFunction)

} // namespace turtlesim
