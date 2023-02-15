#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicCondition.h>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class Master : public BasicPlan
{
public:
    Master(PlanContext& context);
    virtual ~Master();
    static std::unique_ptr<Master> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::Master::create, Master)

class UtilityFunctionMaster : public BasicUtilityFunction
{
public:
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
    static std::shared_ptr<UtilityFunctionMaster> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::UtilityFunctionMaster::create, UtilityFunctionMaster)

} /* namespace alica */
