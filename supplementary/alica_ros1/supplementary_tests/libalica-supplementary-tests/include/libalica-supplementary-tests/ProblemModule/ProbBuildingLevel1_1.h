#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicCondition.h>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class ProbBuildingLevel1_1 : public BasicPlan
{
public:
    ProbBuildingLevel1_1(PlanContext& context);
    virtual ~ProbBuildingLevel1_1();
    static std::unique_ptr<ProbBuildingLevel1_1> create(alica::PlanContext&);
};

BOOST_DLL_ALIAS(alica::ProbBuildingLevel1_1::create, ProbBuildingLevel1_1)

class ProbBuildingLevel1_1UtilityFunction : public BasicUtilityFunction
{
public:
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
    static std::unique_ptr<ProbBuildingLevel1_1UtilityFunction> create(alica::UtilityFunctionContext&);
};

BOOST_DLL_ALIAS(alica::ProbBuildingLevel1_1UtilityFunction::create, ProbBuildingLevel1_1UtilityFunction)
} /* namespace alica */
