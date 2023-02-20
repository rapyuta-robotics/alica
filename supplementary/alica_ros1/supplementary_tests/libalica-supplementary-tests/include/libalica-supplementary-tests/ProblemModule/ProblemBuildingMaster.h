#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicCondition.h>
#include <engine/BasicConstraint.h>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class ProblemDescriptor;

class ProblemBuildingMaster : public BasicPlan
{
public:
    ProblemBuildingMaster(PlanContext& context);
    static std::unique_ptr<ProblemBuildingMaster> create(alica::PlanContext&);
};

BOOST_DLL_ALIAS(alica::ProblemBuildingMaster::create, ProblemBuildingMaster)

class ProblemBuildingMasterUtilityFunction : public BasicUtilityFunction
{
public:
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
    static std::unique_ptr<ProblemBuildingMasterUtilityFunction> create(alica::UtilityFunctionContext&);
};

BOOST_DLL_ALIAS(alica::ProblemBuildingMasterUtilityFunction::create, ProblemBuildingMasterUtilityFunction)
} /* namespace alica */
