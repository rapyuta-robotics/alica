#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace turtlesim
{

class CustomWorkflowsTutorial : public alica::BasicPlan
{
public:
    CustomWorkflowsTutorial(alica::PlanContext& context);
    void onInit() override;
    static std::unique_ptr<CustomWorkflowsTutorial> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(turtlesim::CustomWorkflowsTutorial::create, CustomWorkflowsTutorial)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, CustomWorkflowsTutorialUtilityFunction)

} // namespace turtlesim
