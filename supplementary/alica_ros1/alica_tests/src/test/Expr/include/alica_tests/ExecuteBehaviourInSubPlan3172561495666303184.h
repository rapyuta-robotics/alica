#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class ExecuteBehaviourInSubPlan3172561495666303184 : public DomainPlan
{
public:
    ExecuteBehaviourInSubPlan3172561495666303184(PlanContext& context);
    virtual ~ExecuteBehaviourInSubPlan3172561495666303184();
};

class UtilityFunction3172561495666303184 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition1943478533524176732 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, ExecuteBehaviourInSubPlan3172561495666303184)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, ExecuteBehaviourInSubPlan3172561495666303184UtilityFunction)
} /* namespace alica */
