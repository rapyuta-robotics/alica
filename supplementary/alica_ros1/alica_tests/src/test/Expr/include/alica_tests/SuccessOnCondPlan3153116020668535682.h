#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class SuccessOnCondPlan3153116020668535682 : public DomainPlan
{
public:
    SuccessOnCondPlan3153116020668535682(PlanContext& context);
    virtual ~SuccessOnCondPlan3153116020668535682();

protected:
    virtual void onInit() override;
};

class UtilityFunction3153116020668535682 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition92747471708069515 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, SuccessOnCondPlan3153116020668535682)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, SuccessOnCondPlan3153116020668535682UtilityFunction)

} /* namespace alica */
