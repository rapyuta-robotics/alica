#pragma once

#include "engine/BasicPlan.h"
#include "engine/BasicUtilityFunction.h"
#include <boost/dll/alias.hpp>

namespace alica
{

class Master : public BasicPlan
{
public:
    Master(PlanContext& context);
    virtual ~Master();
    // Factory method
    static std::unique_ptr<Master> create(PlanContext& context) { return std::make_unique<Master>(context); }
};

BOOST_DLL_ALIAS(alica::Master::create, Master)

class MasterUtilityFunction : public BasicUtilityFunction
{
public:
    MasterUtilityFunction() = default;
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan) override;
    // Factory method
    static std::shared_ptr<MasterUtilityFunction> create(UtilityFunctionContext& context)
    {
        (void) context;
        return std::make_shared<MasterUtilityFunction>();
    }
};
BOOST_DLL_ALIAS(alica::MasterUtilityFunction::create, MasterUtilityFunction)
} /* namespace alica */
