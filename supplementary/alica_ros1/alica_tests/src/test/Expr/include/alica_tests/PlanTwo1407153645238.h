#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class PlanTwo1407153645238 : public DomainPlan
{
public:
    PlanTwo1407153645238(PlanContext& context);
    virtual ~PlanTwo1407153645238();
};

class UtilityFunction1407153645238 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, PlanTwo1407153645238)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, PlanTwo1407153645238UtilityFunction)
} /* namespace alica */
