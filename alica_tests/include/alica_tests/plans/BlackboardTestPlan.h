#pragma once

#include <engine/BasicCondition.h>

#include <alica_tests/util/AlicaTestsPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class BlackboardTestPlan : public AlicaTestsPlan<BlackboardTestPlan>
{
public:
    BlackboardTestPlan(PlanContext& context);
    virtual void onInit() override;
};

bool ValueMappingCondition(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);

BOOST_DLL_ALIAS(alica::BlackboardTestPlan::create, BlackboardTestPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, BlackboardTestPlanUtilityFunction)
BOOST_DLL_ALIAS(alica::ValueMappingCondition, ValueMappingCondition)
} /* namespace alica */
