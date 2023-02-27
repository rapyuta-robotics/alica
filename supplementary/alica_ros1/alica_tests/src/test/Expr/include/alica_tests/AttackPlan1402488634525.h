#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class AttackPlan1402488634525 : public DomainPlan
{
public:
    AttackPlan1402488634525(PlanContext& context);
    virtual ~AttackPlan1402488634525();
};

class UtilityFunction1402488634525 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition1402489460549 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition1402489462088 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, AttackPlan1402488634525)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, AttackPlan1402488634525UtilityFunction)
} /* namespace alica */
