#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class FailureHandlingMaster : public alica::BasicPlan
{
public:
    FailureHandlingMaster(alica::PlanContext& context);
    static std::unique_ptr<FailureHandlingMaster> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::FailureHandlingMaster::create, FailureHandlingMaster)

class FailureHandlingMasterUtilityFunction : public alica::BasicUtilityFunction
{
public:
    FailureHandlingMasterUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<FailureHandlingMasterUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::FailureHandlingMasterUtilityFunction::create, FailureHandlingMasterUtilityFunction)

} // namespace alica::tests
