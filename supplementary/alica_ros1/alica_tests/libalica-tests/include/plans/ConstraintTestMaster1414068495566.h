#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class ConstraintTestMaster : public alica::BasicPlan
{
public:
    ConstraintTestMaster(alica::PlanContext& context);
    static std::unique_ptr<ConstraintTestMaster> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::ConstraintTestMaster::create, ConstraintTestMaster)

class ConstraintTestMasterUtilityFunction : public alica::BasicUtilityFunction
{
public:
    ConstraintTestMasterUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<ConstraintTestMasterUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::ConstraintTestMasterUtilityFunction::create, ConstraintTestMasterUtilityFunction)

} // namespace alica::tests
