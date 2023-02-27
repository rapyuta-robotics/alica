#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class HandleFailExplicitMaster1530004940652 : public DomainPlan
{
public:
    HandleFailExplicitMaster1530004940652(PlanContext& context);
    virtual ~HandleFailExplicitMaster1530004940652();
};

class UtilityFunction1530004940652 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, HandleFailExplicitMaster1530004940652)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, HandleFailExplicitMaster1530004940652UtilityFunction)
} /* namespace alica */
