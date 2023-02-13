#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class MasterPlanTaskAssignment : public alica::BasicPlan
{
public:
    MasterPlanTaskAssignment(alica::PlanContext& context);
    static std::unique_ptr<MasterPlanTaskAssignment> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::MasterPlanTaskAssignment::create, MasterPlanTaskAssignment)

class MasterPlanTaskAssignmentUtilityFunction : public alica::BasicUtilityFunction
{
public:
    MasterPlanTaskAssignmentUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<MasterPlanTaskAssignmentUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::MasterPlanTaskAssignmentUtilityFunction::create, MasterPlanTaskAssignmentUtilityFunction)

} // namespace alica::tests
