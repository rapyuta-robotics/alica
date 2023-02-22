#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace turtlesim
{

class SurroundLeaderTutorial : public alica::BasicPlan
{
public:
    SurroundLeaderTutorial(alica::PlanContext& context);
    void onInit() override;
    static std::unique_ptr<SurroundLeaderTutorial> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(turtlesim::SurroundLeaderTutorial::create, SurroundLeaderTutorial)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, SurroundLeaderUtilityFunction)

struct TransitionConditions
{
    static bool CustomAnyChildSuccess(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* globalBlackboard);
};
BOOST_DLL_ALIAS(turtlesim::TransitionConditions::CustomAnyChildSuccess, CustomAnyChildSuccess)

} // namespace turtlesim
