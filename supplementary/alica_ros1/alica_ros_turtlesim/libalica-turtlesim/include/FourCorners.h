#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
#include <engine/logging/Logging.h>

namespace turtlesim
{
class FourCorners : public alica::BasicPlan
{
public:
    FourCorners(alica::PlanContext& context)
            : BasicPlan(context)
    {
    }
};

class FourCornersUtilityFunction : public alica::BasicUtilityFunction
{
public:
    FourCornersUtilityFunction(alica::UtilityFunctionContext& context)
            : alica::BasicUtilityFunction()
    {
    }
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan);
    static std::shared_ptr<FourCornersUtilityFunction> create(alica::UtilityFunctionContext&);
};

BOOST_DLL_ALIAS(turtlesim::FourCorners::create, FourCorners)
BOOST_DLL_ALIAS(turtlesim::FourCornersUtilityFunction::create, FourCornersUtilityFunction)
} /* namespace turtlesim */
