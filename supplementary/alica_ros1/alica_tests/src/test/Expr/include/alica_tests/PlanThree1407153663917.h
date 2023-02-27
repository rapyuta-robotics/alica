#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class PlanThree1407153663917 : public DomainPlan
{
public:
    PlanThree1407153663917(PlanContext& context);
    virtual ~PlanThree1407153663917();
};

class UtilityFunction1407153663917 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, PlanThree1407153663917)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, PlanThree1407153663917UtilityFunction)
} /* namespace alica */
