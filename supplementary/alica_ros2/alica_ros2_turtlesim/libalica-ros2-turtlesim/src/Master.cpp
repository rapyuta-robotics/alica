#include "Master.h"
#include "world_model.hpp"

namespace alica
{
// Plan:  Master (2425328142973735249)
//
// Tasks:
//   - DefaultTask (3310236980587704776) (Entrypoint: 2741715629576575326)
//
// States:
//   - Move (2405597980801916441)
//   - Init (3997532517592149463)
Master::Master(PlanContext& context)
        : BasicPlan(context)
{
}
Master::~Master() {}
std::unique_ptr<Master> Master::create(alica::PlanContext& context)
{
    return std::make_unique<Master>(context);
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 2741715629576575326
 */
std::shared_ptr<UtilityFunction> MasterUtilityFunction::getUtilityFunction(Plan* plan)
{
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<MasterUtilityFunction> MasterUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<MasterUtilityFunction>();
}

} // namespace alica
