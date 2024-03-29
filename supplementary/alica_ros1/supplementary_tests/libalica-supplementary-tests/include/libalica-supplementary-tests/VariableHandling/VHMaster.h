#pragma once

#include <engine/BasicCondition.h>
#include <engine/BasicConstraint.h>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica
{
class ProblemDescriptor;

class VHMaster : public alica::BasicPlan
{
public:
    VHMaster(alica::PlanContext& context);
    static std::unique_ptr<VHMaster> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::VHMaster::create, VHMaster)

class VHMasterUtilityFunction : public alica::BasicUtilityFunction
{
public:
    VHMasterUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<VHMasterUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::VHMasterUtilityFunction::create, VHMasterUtilityFunction)

class VHMasterRuntimeCondition : public BasicCondition
{
public:
    static std::shared_ptr<VHMasterRuntimeCondition> create(ConditionContext& context);
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
BOOST_DLL_ALIAS(alica::VHMasterRuntimeCondition::create, VHMasterRuntimeCondition)

class VHMasterRuntimeConditionConstraint : public BasicConstraint
{
public:
    void getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp);
    static std::shared_ptr<VHMasterRuntimeConditionConstraint> create(alica::ConstraintContext&);
};
BOOST_DLL_ALIAS(alica::VHMasterRuntimeConditionConstraint::create, VHMasterRuntimeConditionConstraint)

} // namespace alica
