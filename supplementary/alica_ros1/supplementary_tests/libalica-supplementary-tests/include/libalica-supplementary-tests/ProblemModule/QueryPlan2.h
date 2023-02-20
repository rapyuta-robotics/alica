#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicCondition.h>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class QueryPlan2 : public BasicPlan
{
public:
    QueryPlan2(PlanContext& context);
    static std::unique_ptr<QueryPlan2> create(alica::PlanContext&);
};

BOOST_DLL_ALIAS(alica::QueryPlan2::create, QueryPlan2)

class QueryPlan2UtilityFunction : public BasicUtilityFunction
{
public:
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
    static std::unique_ptr<QueryPlan2UtilityFunction> create(alica::UtilityFunctionContext&);
};

BOOST_DLL_ALIAS(alica::QueryPlan2UtilityFunction::create, QueryPlan2UtilityFunction)
} /* namespace alica */
