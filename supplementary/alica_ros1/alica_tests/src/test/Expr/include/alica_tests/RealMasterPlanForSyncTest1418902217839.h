#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class RealMasterPlanForSyncTest1418902217839 : public DomainPlan
{
public:
    RealMasterPlanForSyncTest1418902217839(PlanContext& context);
    virtual ~RealMasterPlanForSyncTest1418902217839();
};

class UtilityFunction1418902217839 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, RealMasterPlanForSyncTest1418902217839)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, RealMasterPlanForSyncTest1418902217839UtilityFunction)
} /* namespace alica */
