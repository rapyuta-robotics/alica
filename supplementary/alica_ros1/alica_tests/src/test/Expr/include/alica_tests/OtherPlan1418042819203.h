#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class OtherPlan1418042819203 : public DomainPlan
{
public:
    OtherPlan1418042819203(PlanContext& context);
    virtual ~OtherPlan1418042819203();
};

class UtilityFunction1418042819203 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, OtherPlan1418042819203)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, OtherPlan1418042819203UtilityFunction)
} /* namespace alica */
