#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class MasterSyncTransition1418825395939 : public DomainPlan
{
public:
    MasterSyncTransition1418825395939(PlanContext& context);
    virtual ~MasterSyncTransition1418825395939();
};

class UtilityFunction1418825395939 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition1418825427317 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition1418825428924 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, MasterSyncTransition1418825395939)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, MasterSyncTransition1418825395939UtilityFunction)
} /* namespace alica */
