#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicCondition.h>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class Master2425328142973735249 : public BasicPlan
{
public:
    Master2425328142973735249(PlanContext& context);
    virtual ~Master2425328142973735249();
    static std::unique_ptr<Master2425328142973735249> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::Master2425328142973735249::create, Master2425328142973735249)

class UtilityFunction2425328142973735249 : public BasicUtilityFunction
{
public:
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
    static std::shared_ptr<UtilityFunction2425328142973735249> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::UtilityFunction2425328142973735249::create, UtilityFunction2425328142973735249)

} /* namespace alica */
