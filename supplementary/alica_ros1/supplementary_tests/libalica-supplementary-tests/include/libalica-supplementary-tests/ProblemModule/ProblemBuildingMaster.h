#pragma once

#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
#include <engine/BasicPlan.h>
#include <engine/BasicCondition.h>
#include <engine/BasicConstraint.h>
#include <boost/dll/alias.hpp>

namespace alica
{
class ProblemDescriptor;

class ProblemBuildingMaster : public BasicPlan
{
public:
    ProblemBuildingMaster(PlanContext& context);
    virtual ~ProblemBuildingMaster();
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
