#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class BehaviourTriggerTestPlan1428508768572 : public DomainPlan
{
public:
    BehaviourTriggerTestPlan1428508768572(PlanContext& context);
    virtual ~BehaviourTriggerTestPlan1428508768572();
};

class UtilityFunction1428508768572 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, BehaviourTriggerTestPlan1428508768572)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, BehaviourTriggerTestPlan1428508768572UtilityFunction)
} /* namespace alica */
