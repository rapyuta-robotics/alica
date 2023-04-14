#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class SameBehaviourInParallelTestPlan : public BasicPlan
{
public:
    SameBehaviourInParallelTestPlan(PlanContext& context);
    static std::unique_ptr<SameBehaviourInParallelTestPlan> create(alica::PlanContext& context);
};

BOOST_DLL_ALIAS(alica::SameBehaviourInParallelTestPlan::create, SameBehaviourInParallelTestPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, SameBehaviourInParallelTestPlanUtilityFunction)
} /* namespace alica */
