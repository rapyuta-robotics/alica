#pragma once

#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
#include <engine/BasicPlan.h>
#include <boost/dll/alias.hpp>

namespace alica
{
class GSolverMaster : public BasicPlan
{
public:
    GSolverMaster(PlanContext& context);
    virtual ~GSolverMaster();
    static std::unique_ptr<GSolverMaster> create(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::GSolverMaster::create, GSolverMaster)

class GSolverMasterUtilityFunction : public BasicUtilityFunction
{
public:
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
    static std::unique_ptr<GSolverMasterUtilityFunction> create(UtilityFunctionContext& context);
};

BOOST_DLL_ALIAS(alica::GSolverMasterUtilityFunction::create, GSolverMasterUtilityFunction)
} /* namespace alica */
