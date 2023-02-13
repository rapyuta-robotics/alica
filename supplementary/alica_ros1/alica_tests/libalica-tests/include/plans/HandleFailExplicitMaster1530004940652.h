#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class HandleFailExplicitMaster : public alica::BasicPlan
{
public:
    HandleFailExplicitMaster(alica::PlanContext& context);
    static std::unique_ptr<HandleFailExplicitMaster> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::HandleFailExplicitMaster::create, HandleFailExplicitMaster)

class HandleFailExplicitMasterUtilityFunction : public alica::BasicUtilityFunction
{
public:
    HandleFailExplicitMasterUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<HandleFailExplicitMasterUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::HandleFailExplicitMasterUtilityFunction::create, HandleFailExplicitMasterUtilityFunction)

} // namespace alica::tests
