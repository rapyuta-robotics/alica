#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace turtlesim
{

class AdhocMoveWorkflow : public alica::BasicPlan
{
public:
    AdhocMoveWorkflow(alica::PlanContext& context);
    void onInit() override;
    static std::unique_ptr<AdhocMoveWorkflow> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(turtlesim::AdhocMoveWorkflow::create, AdhocMoveWorkflow)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, AdhocMoveWorkflowUtilityFunction)

} // namespace turtlesim
