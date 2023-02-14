#include <alica/Master2425328142973735249.h>
#include <alica_ros2_turtlesim/world_model.hpp>

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
Master2425328142973735249::Master2425328142973735249(PlanContext& context)
        : BasicPlan(context)
{
}
Master2425328142973735249::~Master2425328142973735249() {}
std::unique_ptr<Master2425328142973735249> Master2425328142973735249::create(alica::PlanContext& context)
{
    return std::make_unique<Master2425328142973735249>(context);
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 2741715629576575326
 */
std::shared_ptr<UtilityFunction> UtilityFunction2425328142973735249::getUtilityFunction(Plan* plan)
{
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<UtilityFunction2425328142973735249> UtilityFunction2425328142973735249::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<UtilityFunction2425328142973735249>();
}

} // namespace alica
