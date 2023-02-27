#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class MultiPlanInstanceSuccessTestPlan3392981108193862307 : public DomainPlan
{
public:
    MultiPlanInstanceSuccessTestPlan3392981108193862307(PlanContext& context);
    virtual ~MultiPlanInstanceSuccessTestPlan3392981108193862307();
};

class UtilityFunction3392981108193862307 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, MultiPlanInstanceSuccessTestPlan3392981108193862307)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, MultiPlanInstanceSuccessTestPlan3392981108193862307UtilityFunction)
} /* namespace alica */
