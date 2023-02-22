#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicCondition.h>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace turtlesim
{
class Master : public alica::BasicPlan
{
public:
    Master(alica::PlanContext& context);
    virtual ~Master();
    static std::unique_ptr<Master> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(turtlesim::Master::create, Master)

class MasterUtilityFunction : public alica::BasicUtilityFunction
{
public:
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan);
    static std::shared_ptr<MasterUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(turtlesim::MasterUtilityFunction::create, MasterUtilityFunction)

} /* namespace turtlesim */
