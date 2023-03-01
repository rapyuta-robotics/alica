#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class FailsOnOne : public DomainPlan
{
public:
    FailsOnOne(PlanContext& context);
};

class RunTimeCondition1530069251117 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, FailsOnOne)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, FailsOnOneUtilityFunction)
} /* namespace alica */
