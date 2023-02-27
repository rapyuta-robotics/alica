#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class SchedulingTestSequenceSubPlan31614964478264 : public DomainPlan
{
public:
    SchedulingTestSequenceSubPlan31614964478264(PlanContext& context);
    virtual ~SchedulingTestSequenceSubPlan31614964478264();
};

class UtilityFunction1614964478264 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, SchedulingTestSequenceSubPlan31614964478264)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, SchedulingTestSequenceSubPlan31614964478264UtilityFunction)

} /* namespace alica */
