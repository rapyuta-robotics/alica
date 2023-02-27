#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class Tackle1402489318663 : public DomainPlan
{
public:
    Tackle1402489318663(PlanContext& context);
    virtual ~Tackle1402489318663();
};

class UtilityFunction1402489318663 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, Tackle1402489318663)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, Tackle1402489318663UtilityFunction)

} /* namespace alica */
