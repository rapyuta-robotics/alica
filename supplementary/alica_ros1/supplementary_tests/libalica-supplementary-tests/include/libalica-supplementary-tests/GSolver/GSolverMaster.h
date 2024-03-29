#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class GSolverMaster : public BasicPlan
{
public:
    GSolverMaster(PlanContext& context);
    static std::unique_ptr<GSolverMaster> create(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::GSolverMaster::create, GSolverMaster)

class GSolverMasterUtilityFunction : public BasicUtilityFunction
{
public:
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
    static std::shared_ptr<GSolverMasterUtilityFunction> create(UtilityFunctionContext& context);
};

BOOST_DLL_ALIAS(alica::GSolverMasterUtilityFunction::create, GSolverMasterUtilityFunction)
} /* namespace alica */
