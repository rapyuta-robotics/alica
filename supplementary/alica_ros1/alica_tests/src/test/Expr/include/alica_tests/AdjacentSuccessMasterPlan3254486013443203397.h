#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class AdjacentSuccessMasterPlan3254486013443203397 : public DomainPlan
{
public:
    AdjacentSuccessMasterPlan3254486013443203397(PlanContext& context);
    virtual ~AdjacentSuccessMasterPlan3254486013443203397();
};

class UtilityFunction3254486013443203397 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition807250359520655888 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition289358204208851392 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, AdjacentSuccessMasterPlan3254486013443203397)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, AdjacentSuccessMasterPlan3254486013443203397UtilityFunction)
} /* namespace alica */
