#include "Master.h"

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
    std::cerr << "Master created" << std::endl;
}
Master::~Master()
{
}


} // namespace alica
