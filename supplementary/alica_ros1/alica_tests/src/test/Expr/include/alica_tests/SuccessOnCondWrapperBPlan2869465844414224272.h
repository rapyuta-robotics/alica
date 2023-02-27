#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class SuccessOnCondWrapperBPlan2869465844414224272 : public DomainPlan
{
public:
    SuccessOnCondWrapperBPlan2869465844414224272(PlanContext& context);
    virtual ~SuccessOnCondWrapperBPlan2869465844414224272();

protected:
    virtual void onInit() override;
};

class UtilityFunction2869465844414224272 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition914907830776317719 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, SuccessOnCondWrapperBPlan2869465844414224272)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, SuccessOnCondWrapperBPlan2869465844414224272UtilityFunction)

} /* namespace alica */
