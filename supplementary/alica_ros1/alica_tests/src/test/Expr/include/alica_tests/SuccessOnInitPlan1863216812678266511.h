#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class SuccessOnInitPlan1863216812678266511 : public DomainPlan
{
public:
    SuccessOnInitPlan1863216812678266511(PlanContext& context);
    virtual ~SuccessOnInitPlan1863216812678266511();
};

class UtilityFunction1863216812678266511 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition4197030928062612573 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, SuccessOnInitPlan1863216812678266511)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, SuccessOnInitPlan1863216812678266511UtilityFunction)
} /* namespace alica */
