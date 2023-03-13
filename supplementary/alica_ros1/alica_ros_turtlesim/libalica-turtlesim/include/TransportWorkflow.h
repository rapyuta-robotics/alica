#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace turtlesim
{

class TransportWorkflow : public alica::BasicPlan
{
public:
    TransportWorkflow(alica::PlanContext& context);
    void onInit() override;
    static std::unique_ptr<TransportWorkflow> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(turtlesim::TransportWorkflow::create, TransportWorkflow)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, TransportWorkflowUtilityFunction)

} // namespace turtlesim
