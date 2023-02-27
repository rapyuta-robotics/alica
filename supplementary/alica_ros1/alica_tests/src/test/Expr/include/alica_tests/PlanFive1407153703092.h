#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class PlanFive1407153703092 : public DomainPlan
{
public:
    PlanFive1407153703092(PlanContext& context);
    virtual ~PlanFive1407153703092();
};

class UtilityFunction1407153703092 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, PlanFive1407153703092)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, PlanFive1407153703092UtilityFunction)
} /* namespace alica */
