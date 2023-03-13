#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica_standard_library
{

class UntracedPlan : public alica::BasicPlan
{
public:
    UntracedPlan(alica::PlanContext& context);
    static std::unique_ptr<UntracedPlan> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica_standard_library::UntracedPlan::create, UntracedPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, UntracedPlanUtilityFunction)

class TracedPlan : public alica::BasicPlan
{
public:
    TracedPlan(alica::PlanContext& context);
    static std::unique_ptr<TracedPlan> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica_standard_library::TracedPlan::create, TracedPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, TracedPlanUtilityFunction)

} // namespace alica_standard_library
