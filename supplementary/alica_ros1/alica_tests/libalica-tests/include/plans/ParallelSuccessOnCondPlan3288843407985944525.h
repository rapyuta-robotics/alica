#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class PLANNAME : public alica::BasicPlan
{
public:
    PLANNAME(alica::PlanContext& context);
    static std::unique_ptr<PLANNAME> create(alica::PlanContext& context);
    virtual void onInit() override;
};
BOOST_DLL_ALIAS(alica::tests::PLANNAME::create, PLANNAME)

class PLANNAMEUtilityFunction : public alica::BasicUtilityFunction
{
public:
    PLANNAMEUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<PLANNAMEUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::PLANNAMEUtilityFunction::create, PLANNAMEUtilityFunction)

} // namespace alica::tests
