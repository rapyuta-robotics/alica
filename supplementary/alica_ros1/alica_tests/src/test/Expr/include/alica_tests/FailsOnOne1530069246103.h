#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class FailsOnOne1530069246103 : public DomainPlan
{
public:
    FailsOnOne1530069246103(PlanContext& context);
    virtual ~FailsOnOne1530069246103();
};

class UtilityFunction1530069246103 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class RunTimeCondition1530069251117 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, FailsOnOne1530069246103)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, FailsOnOne1530069246103UtilityFunction)
} /* namespace alica */
